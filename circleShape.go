package chipmunk

import (
	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"
)

type CircleShape struct {
	Shape *Shape
	// Center of the circle. Call Update() on the parent shape if changed.
	Position vect.Vect
	// Radius of the circle. Call Update() on the parent shape if changed.
	Radius vect.Float
	// Global center of the circle. Do not touch!
	Tc vect.Vect
}

// Creates a new CircleShape with the given center and radius.
func NewCircle(pos vect.Vect, radius float32) *Shape {
	shape := newShape()
	circle := &CircleShape{
		Position: pos,
		Radius:   vect.Float(radius),
		Shape:    shape,
	}
	shape.ShapeClass = circle
	return shape
}

// Returns ShapeType_Circle. Needed to implemet the ShapeClass interface.
func (circle *CircleShape) ShapeType() ShapeType {
	return ShapeType_Circle
}

func (circle *CircleShape) Moment(mass float32) vect.Float {
	return (vect.Float(mass) * (0.5 * (circle.Radius * circle.Radius))) + vect.LengthSqr(circle.Position)
}

// Recalculates the global center of the circle and the the bounding box.
func (circle *CircleShape) update(xf transform.Transform) AABB {
	//global center of the circle
	center := xf.TransformVect(circle.Position)
	circle.Tc = center
	rv := vect.Vect{circle.Radius, circle.Radius}

	return AABB{
		vect.Sub(center, rv),
		vect.Add(center, rv),
	}
}

// Returns ShapeType_Box. Needed to implemet the ShapeClass interface.
func (circle *CircleShape) Clone(s *Shape) ShapeClass {
	clone := *circle
	clone.Shape = s
	return &clone
}

// Returns true if the given point is located inside the circle.
func (circle *CircleShape) TestPoint(point vect.Vect) bool {
	d := vect.Sub(point, circle.Tc)

	return vect.Dot(d, d) <= circle.Radius*circle.Radius
}
