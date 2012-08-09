package chipmunk

import (
	. "github.com/vova616/chipmunk/vect"
	"github.com/vova616/chipmunk/transform"
	"math"
	//"fmt"
)

const (
	RadianConst = math.Pi / 180
	DegreeConst = 180 / math.Pi
)

type Shape struct{
	DefaultHash
	ShapeClass
	
	/// The rigid body this collision shape is attached to.
	Body *Body;

	/// The current bounding box of the shape.
	BB AABB;
	
	/// Sensor flag.
	/// Sensor shapes call collision callbacks but don't produce collisions.
	IsSensor bool
	
	/// Coefficient of restitution. (elasticity)
	e Float;
	/// Coefficient of friction.
	u Float;
	/// Surface velocity used when solving for friction.
	Surface_v Vect; 

	/// User definable data pointer.
	/// Generally this points to your the game object class so you can access it
	/// when given a cpShape reference in a callback.
	UserData interface{};
	
	/// Collision type of this shape used when picking collision handlers.
	//collision_type CollisionType
	/// Group of this shape. Shapes in the same group don't collide.
	//group Group;
	// Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
	//layers Layers;
	
	space *Space
	
	
	velocityIndexed bool
};

func newShape() *Shape {
	return &Shape{velocityIndexed: true, e: 0.5, u: 0.5}
	
}

func (shape *Shape) Velocity() (Vect,bool) {
	return shape.Body.v,shape.velocityIndexed
}



func (shape *Shape) SetFriction(friction Float) {
	shape.u = friction
}

func (shape *Shape) SetElasticity(e Float) {
	shape.e = e
}

func (shape *Shape) Shape() *Shape {
	return shape
}


func (shape *Shape) AABB()  AABB {
	return shape.BB
}
 
func (shape *Shape) Update()  {
	//fmt.Println("Rot", shape.Body.rot)
	shape.BB = shape.ShapeClass.update(transform.NewTransform(shape.Body.p, shape.Body.a))
}
