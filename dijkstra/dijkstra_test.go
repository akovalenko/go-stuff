package dijkstra

import (
	"math"
	"strings"
	"testing"
)

// Point is a [row, col] coordinate used as a graph vertex.
type Point [2]int

// labyrinth is a small maze: S is the start, T marks targets,
// # are walls, spaces are open passages.
// The two Ts sit at distances 5 and 7 from S.
// The only route to T2 goes through T1, so TargetAny and TargetAll
// exhibit observably different stopping behaviour.
var labyrinth = `
#######
#S    #
## ## #
## ## #
#  T  #
### ###
#  T  #
#######`

func parseBoard(s string) []string {
	lines := strings.Split(s, "\n")
	if lines[0] == "" {
		lines = lines[1:]
	}
	return lines
}

func findStart(board []string) Point {
	for r, row := range board {
		for c, ch := range row {
			if ch == 'S' {
				return Point{r, c}
			}
		}
	}
	panic("no S in board")
}

func findTargets(board []string) []Point {
	var pts []Point
	for r, row := range board {
		for c, ch := range row {
			if ch == 'T' {
				pts = append(pts, Point{r, c})
			}
		}
	}
	return pts
}

// boardNeighbors returns a neighbors function for Dijkstra that yields
// the four cardinal non-wall cells within bounds, each at distance 1.
func boardNeighbors(board []string) func(Point) ([]Point, []int) {
	rows := len(board)
	cols := len(board[0])
	return func(p Point) ([]Point, []int) {
		var ns []Point
		var ds []int
		for _, d := range [4][2]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}} {
			r, c := p[0]+d[0], p[1]+d[1]
			if r >= 0 && r < rows && c >= 0 && c < cols && board[r][c] != '#' {
				ns = append(ns, Point{r, c})
				ds = append(ds, 1)
			}
		}
		return ns, ds
	}
}

// TestLabyrinthTargetAny verifies that Dijkstra stops at the nearest
// target (T1 at distance 5). Because T2 is only reachable through T1,
// it must not appear in the settled set.
func TestLabyrinthTargetAny(t *testing.T) {
	board := parseBoard(labyrinth)
	start := findStart(board)
	targets := findTargets(board)
	t1, t2 := targets[0], targets[1]

	dists, _, err := Dijkstra(start, boardNeighbors(board), TargetAny(targets...))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if d, ok := dists[t1]; !ok || d != 5 {
		t.Errorf("T1 %v: expected dist 5, got dist=%d ok=%v", t1, d, ok)
	}
	if _, ok := dists[t2]; ok {
		t.Errorf("T2 %v should not be settled when TargetAny stops at T1", t2)
	}
}

// TestLabyrinthTargetAll verifies that Dijkstra runs until both targets
// are settled, and that BuildRoute produces valid paths of matching length.
func TestLabyrinthTargetAll(t *testing.T) {
	board := parseBoard(labyrinth)
	start := findStart(board)
	targets := findTargets(board)

	dists, prevs, err := Dijkstra(start, boardNeighbors(board), TargetAll(targets...))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	wantDist := []int{5, 7}
	for i, target := range targets {
		d, ok := dists[target]
		if !ok {
			t.Errorf("target %v not reached", target)
			continue
		}
		if d != wantDist[i] {
			t.Errorf("target %v: expected dist %d, got %d", target, wantDist[i], d)
		}
		route := BuildRoute(start, target, prevs)
		if route == nil {
			t.Errorf("BuildRoute returned nil for target %v", target)
			continue
		}
		if route[0] != start || route[len(route)-1] != target {
			t.Errorf("route endpoints wrong for %v: %v", target, route)
		}
		if len(route)-1 != d {
			t.Errorf("route length %d != distance %d for target %v", len(route)-1, d, target)
		}
	}
}

func TestAddOverflowUint8(t *testing.T) {
	if _, ov := AddOverflow[uint8](200, 100); !ov {
		t.Error("expected overflow: uint8(200+100)")
	}
	if sum, ov := AddOverflow[uint8](100, 100); ov || sum != 200 {
		t.Errorf("expected sum=200 no overflow, got sum=%d overflow=%v", sum, ov)
	}
}

func TestAddOverflowInt8(t *testing.T) {
	if _, ov := AddOverflow[int8](100, 100); !ov {
		t.Error("expected overflow: int8(100+100)")
	}
	if _, ov := AddOverflow[int8](-100, -100); !ov {
		t.Error("expected overflow: int8(-100+(-100))")
	}
	if sum, ov := AddOverflow[int8](50, 50); ov || sum != 100 {
		t.Errorf("expected sum=100 no overflow, got sum=%d overflow=%v", sum, ov)
	}
}

func TestAddOverflowFloat64(t *testing.T) {
	if _, ov := AddOverflow(math.Inf(1), 1.0); !ov {
		t.Error("expected overflow: +Inf + 1.0")
	}
	if _, ov := AddOverflow(math.MaxFloat64, math.MaxFloat64); !ov {
		t.Error("expected overflow: MaxFloat64 + MaxFloat64 -> +Inf")
	}
	if sum, ov := AddOverflow(1.0, 2.0); ov || sum != 3.0 {
		t.Errorf("expected sum=3.0 no overflow, got sum=%f overflow=%v", sum, ov)
	}
}

func TestTargetAllVacuous(t *testing.T) {
	f := TargetAll[int]()
	if !f(42) {
		t.Error("TargetAll() with no targets should return true immediately (vacuous truth)")
	}
}

func TestTargetAnyNever(t *testing.T) {
	f := TargetAny[int]()
	if f(0) || f(1) {
		t.Error("TargetAny() with no targets should never match")
	}
}

func TestErrNegativeDistance(t *testing.T) {
	neighbors := func(int) ([]int, []int) {
		return []int{1}, []int{-1}
	}
	if _, _, err := Dijkstra(0, neighbors, nil); err != ErrNegativeDistance {
		t.Errorf("expected ErrNegativeDistance, got %v", err)
	}
}

func TestErrNeighborsMismatch(t *testing.T) {
	neighbors := func(int) ([]int, []int) {
		return []int{1, 2}, []int{1}
	}
	if _, _, err := Dijkstra(0, neighbors, nil); err != ErrNeighborsMismatch {
		t.Errorf("expected ErrNeighborsMismatch, got %v", err)
	}
}
