package vect

import ( 
	"math"
	"testing"
)

type addTest struct {
	in1, in2 Vect
	out      Vect
}

var addTests = []addTest{
	{Vect{0, 0}, Vect{0, 0}, Vect{0, 0}},
	{Vect{0, 1}, Vect{0, 0}, Vect{0, 1}},
	{Vect{1, 0}, Vect{0, 0}, Vect{1, 0}},
	{Vect{1, 2}, Vect{0, 0}, Vect{1, 2}},
	{Vect{0, 0}, Vect{0, 1}, Vect{0, 1}},
	{Vect{0, 0}, Vect{1, 0}, Vect{1, 0}},
	{Vect{0, 0}, Vect{1, 2}, Vect{1, 2}},
	{Vect{2, 4}, Vect{1, 3}, Vect{3, 7}},
	{Vect{3, 1}, Vect{4, 2}, Vect{7, 3}},
	{Vect{2, 4}, Vect{2, 4}, Vect{4, 8}},
	{Vect{5, 5}, Vect{2, 2}, Vect{7, 7}},
}

func TestAdd(t *testing.T) {
	for _, at := range addTests {
		v := Add(at.in1, at.in2)
		if !Equals(at.out, v) {
			t.Errorf("Add(%v, %v) = %v, want %v.", at.in1, at.in2, v, at.out)
		}
	}
}

type minTest struct {
	in1, in2 Vect
	out      Vect
}

var minTests = []minTest{
	{Vect{0, 0}, Vect{0, 0}, Vect{0, 0}},
	{Vect{1, 2}, Vect{9, 9}, Vect{1, 2}},
	{Vect{9, 9}, Vect{1, 2}, Vect{1, 2}},
	{Vect{5, 2}, Vect{1, 4}, Vect{1, 2}},
	{Vect{9, 6}, Vect{7, 8}, Vect{7, 6}},
}

func TestMin(t *testing.T) {
	for _, at := range minTests {
		v := Min(at.in1, at.in2)
		if !Equals(at.out, v) {
			t.Errorf("Min(%v, %v) = %v, want %v.", at.in1, at.in2, v, at.out)
		}
	}
}

type maxTest struct {
	in1, in2 Vect
	out      Vect
}

var maxTests = []maxTest{
	{Vect{0, 0}, Vect{0, 0}, Vect{0, 0}},
	{Vect{1, 2}, Vect{9, 9}, Vect{9, 9}},
	{Vect{9, 9}, Vect{1, 2}, Vect{9, 9}},
	{Vect{5, 2}, Vect{1, 4}, Vect{5, 4}},
	{Vect{9, 6}, Vect{7, 8}, Vect{9, 8}},
}

func TestMax(t *testing.T) {
	for _, at := range maxTests {
		v := Max(at.in1, at.in2)
		if !Equals(at.out, v) {
			t.Errorf("Max(%v, %v) = %v, want %v.", at.in1, at.in2, v, at.out)
		}
	}
}

type distTest struct {
	in1, in2 Vect
	out      float64
}

var distTests = []distTest{
	{Vect{0, 0}, Vect{0, 0}, 0},
	{Vect{0, 2}, Vect{0, 0}, 2},
	{Vect{2, 0}, Vect{0, 0}, 2},
	{Vect{0, 0}, Vect{4, 0}, 4},
	{Vect{0, 0}, Vect{0, 4}, 4},
	{Vect{1, 1}, Vect{0, 0}, math.Sqrt(2)},
	{Vect{1, 1}, Vect{2, 2}, math.Sqrt(2)},
}

func TestDist(t *testing.T) {
	for _, at := range distTests {
		v := Dist(at.in1, at.in2)
		if at.out != v {
			t.Errorf("Dist(%v, %v) = %v, want %v.", at.in1, at.in2, v, at.out)
		}
	}
}
