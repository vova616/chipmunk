package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
)

//axis aligned bounding box.
type AABB struct {
	Lower, //l b
	Upper vect.Vect // r t
}

/*
	l := aabb.Lower.X
	b := aabb.Lower.Y
	r := aabb.Upper.X
	t := aabb.Upper.Y
*/

func (aabb *AABB) Valid() bool {
	return aabb.Lower.X <= aabb.Upper.X && aabb.Lower.Y <= aabb.Upper.Y
}

func NewAABB(l, b, r, t vect.Float) AABB {
	return AABB{vect.Vect{l, b}, vect.Vect{r, t}}
}

//returns the center of the aabb
func (aabb *AABB) Center() vect.Vect {
	return vect.Mult(vect.Add(aabb.Lower, aabb.Upper), 0.5)
}

//returns if other is contained inside this aabb.
func (aabb *AABB) Contains(other AABB) bool {
	return aabb.Lower.X <= other.Lower.X &&
		aabb.Upper.X >= other.Upper.X &&
		aabb.Lower.Y <= other.Lower.Y &&
		aabb.Upper.Y >= other.Upper.Y
}

//returns if other is contained inside this aabb.
func (aabb *AABB) ContainsPtr(other *AABB) bool {
	return aabb.Lower.X <= other.Lower.X &&
		aabb.Upper.X >= other.Upper.X &&
		aabb.Lower.Y <= other.Lower.Y &&
		aabb.Upper.Y >= other.Upper.Y
}

//returns if v is contained inside this aabb.
func (aabb *AABB) ContainsVect(v vect.Vect) bool {
	return aabb.Lower.X <= v.X &&
		aabb.Upper.X >= v.X &&
		aabb.Lower.Y <= v.Y &&
		aabb.Upper.Y >= v.Y
}

func (aabb *AABB) Extents() vect.Vect {
	return vect.Mult(vect.Sub(aabb.Upper, aabb.Lower), .5)
}

func (aabb *AABB) Perimeter() vect.Float {
	w := vect.Sub(aabb.Upper, aabb.Lower)
	return 2 * (w.X + w.Y)
}

//returns an AABB that holds both a and b.
func Combine(a, b AABB) AABB {
	return AABB{
		vect.Min(a.Lower, b.Lower),
		vect.Max(a.Upper, b.Upper),
	}
}

//returns an AABB that holds both a and b.
func CombinePtr(a, b *AABB) AABB {
	return AABB{
		vect.Min(a.Lower, b.Lower),
		vect.Max(a.Upper, b.Upper),
	}
}

//returns an AABB that holds both a and v.
func Expand(a AABB, v vect.Vect) AABB {
	return AABB{
		vect.Min(a.Lower, v),
		vect.Max(a.Upper, v),
	}
}

//returns the area of the bounding box.
func (aabb *AABB) Area() vect.Float {
	return (aabb.Upper.X - aabb.Lower.X) * (aabb.Upper.Y - aabb.Lower.Y)
}

func MergedArea(a, b AABB) vect.Float {
	return (vect.FMax(a.Upper.X, b.Upper.X) - vect.FMin(a.Lower.X, b.Lower.X)) * (vect.FMax(a.Upper.Y, b.Upper.Y) - vect.FMin(a.Lower.Y, b.Lower.Y))
}

func MergedAreaPtr(a, b *AABB) vect.Float {
	return (vect.FMax(a.Upper.X, b.Upper.X) - vect.FMin(a.Lower.X, b.Lower.X)) * (vect.FMax(a.Upper.Y, b.Upper.Y) - vect.FMin(a.Lower.Y, b.Lower.Y))
}

func ProximityPtr(a, b *AABB) vect.Float {
	return vect.FAbs(a.Lower.X+a.Upper.X-b.Lower.X-b.Upper.X) + vect.FAbs(a.Lower.Y+a.Upper.Y-b.Lower.Y-b.Upper.Y)
}

func Proximity(a, b AABB) vect.Float {
	return vect.FAbs(a.Lower.X+a.Upper.X-b.Lower.X-b.Upper.X) + vect.FAbs(a.Lower.Y+a.Upper.Y-b.Lower.Y-b.Upper.Y)
}

func TestOverlap2(a, b AABB) bool {

	d1 := vect.Sub(b.Lower, a.Upper)
	d2 := vect.Sub(a.Lower, b.Upper)

	if d1.X > 0.0 || d1.Y > 0.0 {
		return false
	}

	if d2.X > 0.0 || d2.Y > 0.0 {
		return false
	}

	return true
}

func TestOverlap(a, b AABB) bool {
	return (a.Lower.X <= b.Upper.X && b.Lower.X <= a.Upper.X && a.Lower.Y <= b.Upper.Y && b.Lower.Y <= a.Upper.Y)
}

func TestOverlapPtr(a, b *AABB) bool {
	return (a.Lower.X <= b.Upper.X && b.Lower.X <= a.Upper.X && a.Lower.Y <= b.Upper.Y && b.Lower.Y <= a.Upper.Y)
}
