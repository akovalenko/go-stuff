package dijkstra

import (
	"container/heap"
	"errors"
	"slices"
)

type Numeric interface {
	~uint64 | ~uint32 | ~uint16 | ~uint8 | ~uint | ~uintptr |
		~int64 | ~int32 | ~int16 | ~int8 | ~int |
		~float64 | ~float32
}

type item[K comparable, D Numeric] struct {
	key  K
	dist D
}

// indexedHeapStore implements heap.Interface while keeping track of
// keyed item positions in pos
type indexedHeapStore[K comparable, D Numeric] struct {
	items []*item[K, D]
	pos   map[K]int
}

// Len, Less, Swap, Push, Pop from heap.Interface

func (ihs *indexedHeapStore[K, D]) Len() int {
	return len(ihs.items)
}

func (ihs *indexedHeapStore[K, D]) Less(i, j int) bool {
	return ihs.items[i].dist < ihs.items[j].dist
}

func (ihs *indexedHeapStore[K, D]) Swap(i, j int) {
	ihs.items[i], ihs.items[j] =
		ihs.items[j], ihs.items[i]
	ihs.pos[ihs.items[i].key] = i
	ihs.pos[ihs.items[j].key] = j
}

func (ihs *indexedHeapStore[K, D]) Push(v any) {
	item := v.(*item[K, D])
	pos := len(ihs.items)
	ihs.items = append(ihs.items, item)
	ihs.pos[item.key] = pos
}

func (ihs *indexedHeapStore[K, D]) Pop() any {
	pos := len(ihs.items) - 1
	var result = ihs.items[pos]
	ihs.items[pos] = nil
	ihs.items = ihs.items[0:pos]
	delete(ihs.pos, result.key)
	return result
}

// IndexedHeap stores keys of type K with priorities of type D.
// Specially designed for Dijkstra algorithm.
type IndexedHeap[K comparable, D Numeric] struct {
	store indexedHeapStore[K, D]
}

// ExtractMin pops the minimal-distance item, returning key and
// distance
func (ih *IndexedHeap[K, D]) ExtractMin() (K, D) {
	item := heap.Pop(&ih.store).(*item[K, D])
	return item.key, item.dist
}

// ReducePriority either adds an item (if it is not there) or lowers
// existing item's distance if given distance is less then the old
// one. Returns true iff the distance was indeed reduced.
// Conceptually, the distance to an item which is not there is
// infinity (hence reduced when the item is added).
func (ih *IndexedHeap[K, D]) ReducePriority(key K, distance D) bool {
	if pos, ok := ih.store.pos[key]; ok {
		if ih.store.items[pos].dist > distance {
			ih.store.items[pos].dist = distance
			heap.Fix(&ih.store, pos)
			return true
		}
		return false
	} else {
		heap.Push(&ih.store, &item[K, D]{key: key, dist: distance})
		return true
	}
}

// Len returns item count for IndexedHeap
func (ih *IndexedHeap[K, D]) Len() int {
	return ih.store.Len()
}

// NewIndexedHeap creates and returns a new, empty IndexedHeap
func NewIndexedHeap[K comparable, D Numeric]() *IndexedHeap[K, D] {
	return &IndexedHeap[K, D]{
		store: indexedHeapStore[K, D]{
			pos: make(map[K]int),
		},
	}
}

// AddOverflow adds two numbers with overflow detection.  For floats,
// it also returns overflow==true if there is a NaN input, which is
// deemed right in the context of Dijkstra.
func AddOverflow[T Numeric](a, b T) (sum T, overflow bool) {
	sum = a + b
	if sum-sum != 0 {
		// overflow for floats
		return sum, true
	}
	if b >= 0 {
		// for unsigned, this branch is always taken
		overflow = sum < a
	} else {
		// for signed negative b, sum > a means wraparound
		overflow = sum > a
	}
	return
}

var (
	ErrOverflow          = errors.New("Distance addition overflow")
	ErrNegativeDistance  = errors.New("Negative distance")
	ErrNeighborsMismatch = errors.New("Neighbors slice lengths do not match")
)

// Dijkstra runs Dijkstra's shortest path finding against a graph,
// from the starting point start to all graph's vertices or to the
// nearest target detected by optional isTarget function.
//
// To avoid requiring any particular graph representation, it accepts
// the neighbors function, that should return all vertexes directly
// reachable from the argument together with distances to them, in
// parallel slices.
//
// The algorithm stops when there are no new vertexes to reach, or
// when a non-nil isTarget returns true for a vertex with known
// shortest distance.
//
// Return values: map[K]D mapping each vertex encountered by the
// algorithm along the way to the shortest path from start to this
// vertex, map[K]K mapping each vertex (except start) to the previous
// vertex along one of the shortest paths, allowing to build an
// example shortest path from start to any vertex encountered.
//
// On errors, ErrNeighborsMismatch, ErrNegativeDistance, ErrOverflow
// can be returned (with nil maps)
func Dijkstra[K comparable, D Numeric](start K, neighbors func(K) ([]K, []D),
	isTarget func(K) bool) (map[K]D, map[K]K, error) {
	ih := NewIndexedHeap[K, D]()
	ih.ReducePriority(start, 0)
	result := map[K]D{}
	prevs := map[K]K{}

	for ih.Len() > 0 {
		here, dist := ih.ExtractMin()
		result[here] = dist
		if isTarget != nil && isTarget(here) {
			break
		}
		neighs, dists := neighbors(here)
		if len(neighs) != len(dists) {
			return nil, nil, ErrNeighborsMismatch
		}

		for i := range neighs {
			if _, ok := result[neighs[i]]; !ok {
				delta := dists[i]
				if delta < 0 {
					return nil, nil, ErrNegativeDistance
				}
				newDist, overflow := AddOverflow(dist, delta)
				if overflow {
					return nil, nil, ErrOverflow
				}
				if ih.ReducePriority(neighs[i], newDist) {
					prevs[neighs[i]] = here
				}
			}
		}
	}
	return result, prevs, nil
}

// TargetAny creates a target-filtering closure for Dijkstra,
// returning true when the argument is present within a slice
func TargetAny[K comparable](target ...K) func(K) bool {
	m := map[K]struct{}{}
	for _, item := range target {
		m[item] = struct{}{}
	}
	return func(item K) bool {
		_, present := m[item]
		return present
	}
}

// TargetAll creates a target-filtering closure for Dijkstra,
// returning true when all specified targets were reached. Mutates a
// shared state inside, so should constructed once per call to
// Dijkstra.
//
// Note: with no arguments, the closure returns true immediately, so
// Dijkstra will stop after settling the start node. This follows from
// vacuous truth but may be surprising.
func TargetAll[K comparable](target ...K) func(K) bool {
	m := map[K]struct{}{}
	for _, item := range target {
		m[item] = struct{}{}
	}
	return func(item K) bool {
		delete(m, item)
		return len(m) == 0
	}
}

// BuildRoute takes a "prevs" map from Dijkstra and builds a list of
// nodes starting at start, ending at end, which is a possible
// shortest path from start to end. If there is no way to reach start
// from end following links in prevs[], BuildRoute returns nil.
func BuildRoute[K comparable](start, end K, prevs map[K]K) []K {
	here := end
	var route []K
	for {
		route = append(route, here)
		if here == start {
			slices.Reverse(route)
			return route
		}
		var ok bool
		here, ok = prevs[here]
		if !ok {
			return nil
		}
	}
}
