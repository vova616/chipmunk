package chipmunk

import (
	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"
	"math"
	//"fmt"
)

const (
	RadianConst = math.Pi / 180
	DegreeConst = 180 / math.Pi
)

type Group int
type Layer int

type Shape struct {
	DefaultHash
	ShapeClass

	/// The rigid body this collision shape is attached to.
	Body *Body

	/// The current bounding box of the shape.
	BB AABB

	/// Sensor flag.
	/// Sensor shapes call collision callbacks but don't produce collisions.
	IsSensor bool

	/// Coefficient of restitution. (elasticity)
	e vect.Float
	/// Coefficient of friction.
	u vect.Float
	/// Surface velocity used when solving for friction.
	Surface_v vect.Vect

	/// User definable data pointer.
	/// Generally this points to your the game object class so you can access it
	/// when given a cpShape reference in a callback.
	UserData interface{}

	/// Collision type of this shape used when picking collision handlers.
	//collision_type CollisionType
	/// Group of this shape. Shapes in the same group don't collide.
	Group Group
	// Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
	Layer Layer

	space *Space

	velocityIndexed bool
}

func newShape() *Shape {
	return &Shape{velocityIndexed: true, e: 0.5, u: 0.5, Layer: -1}

}

func (shape *Shape) Velocity() (vect.Vect, bool) {
	return shape.Body.v, shape.velocityIndexed
}

func (shape *Shape) SetFriction(friction vect.Float) {
	shape.u = friction
}

func (shape *Shape) SetElasticity(e vect.Float) {
	shape.e = e
}

func (shape *Shape) Shape() *Shape {
	return shape
}

func (shape *Shape) AABB() AABB {
	return shape.BB
}

func (shape *Shape) Clone() *Shape {
	clone := *shape
	cc := &clone
	cc.space = nil
	cc.DefaultHash.Reset()
	cc.Body = nil
	cc.ShapeClass = cc.ShapeClass.Clone(cc)
	return cc
}

func (shape *Shape) Update() {
	//fmt.Println("Rot", shape.Body.rot)
	shape.BB = shape.ShapeClass.update(transform.NewTransform(shape.Body.p, shape.Body.a))
}
