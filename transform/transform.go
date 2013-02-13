package transform

import (
	"github.com/vova616/chipmunk/vect"
	"math"
)

type Rotation struct {
	//sine and cosine.
	C, S vect.Float
}

func NewRotation(angle vect.Float) Rotation {
	return Rotation{
		C: vect.Float(math.Cos(float64(angle))),
		S: vect.Float(math.Sin(float64(angle))),
	}
}

func (rot *Rotation) SetIdentity() {
	rot.S = 0
	rot.C = 1
}

func (rot *Rotation) SetAngle(angle vect.Float) {
	rot.C = vect.Float(math.Cos(float64(angle)))
	rot.S = vect.Float(math.Sin(float64(angle)))
}

func (rot *Rotation) Angle() vect.Float {
	return vect.Float(math.Atan2(float64(rot.S), float64(rot.C)))
}

//rotates the input vector.
func (rot *Rotation) RotateVect(v vect.Vect) vect.Vect {
	return vect.Vect{
		X: (v.X * rot.C) - (v.Y * rot.S),
		Y: (v.X * rot.S) + (v.Y * rot.C),
	}
}

//rotates the input vector.
func (rot *Rotation) RotateVectPtr(v *vect.Vect) vect.Vect {
	return vect.Vect{
		X: (v.X * rot.C) - (v.Y * rot.S),
		Y: (v.X * rot.S) + (v.Y * rot.C),
	}
}

func (rot *Rotation) RotateVectInv(v vect.Vect) vect.Vect {
	return vect.Vect{
		X: (v.X * rot.C) + (v.Y * rot.S),
		Y: (-v.X * rot.S) + (v.Y * rot.C),
	}
}

func RotateVect(v vect.Vect, r Rotation) vect.Vect {
	return r.RotateVect(v)
}

func RotateVectPtr(v *vect.Vect, r *Rotation) vect.Vect {
	return r.RotateVectPtr(v)
}

func RotateVectInv(v vect.Vect, r Rotation) vect.Vect {
	return r.RotateVectInv(v)
}

type Transform struct {
	Position vect.Vect
	Rotation
}

func NewTransform(pos vect.Vect, angle vect.Float) Transform {
	return Transform{
		Position: pos,
		Rotation: NewRotation(angle),
	}
}

func NewTransform2(pos vect.Vect, rot vect.Vect) Transform {
	return Transform{
		Position: pos,
		Rotation: Rotation{rot.X, rot.Y},
	}
}

func (xf *Transform) SetIdentity() {
	xf.Position = vect.Vect{}
	xf.Rotation.SetIdentity()
}

func (xf *Transform) Set(pos vect.Vect, rot vect.Float) {
	xf.Position = pos
	xf.SetAngle(rot)
}

//moves and roates the input vector.
func (xf *Transform) TransformVect(v vect.Vect) vect.Vect {
	return vect.Add(xf.Position, xf.RotateVect(v))
}

func (xf *Transform) TransformVectInv(v vect.Vect) vect.Vect {
	return vect.Add(vect.Mult(xf.Position, -1), xf.RotateVectInv(v))
}
