package chipmunk

import (
	//"fmt"
	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"
	"time"
)

// Used to keep a linked list of all arbiters on a body.
type ArbiterEdge struct {
	Arbiter    *Arbiter
	Next, Prev *ArbiterEdge
	Other      *Body
}

type arbiterState int

const (
	arbiterStateFirstColl = iota
	arbiterStateNormal
)

// The maximum number of ContactPoints a single Arbiter can have.
const MaxPoints = 4

type Arbiter struct {
	// The two colliding shapes.
	ShapeA, ShapeB *Shape
	// The contact points between the shapes.
	Contacts *[MaxPoints]*Contact
	// The number of contact points.
	NumContacts int

	nodeA, nodeB *ArbiterEdge

	/// Calculated value to use for the elasticity coefficient.
	/// Override in a pre-solve collision handler for custom behavior.
	e vect.Float
	/// Calculated value to use for the friction coefficient.
	/// Override in a pre-solve collision handler for custom behavior.
	u vect.Float
	 /// Calculated value to use for applying surface velocities.
	/// Override in a pre-solve collision handler for custom behavior.
	Surface_vr vect.Vect

	// Used to keep a linked list of all arbiters in a space.
	Next, Prev *Arbiter

	state arbiterState
	stamp time.Duration
}

func newArbiter() *Arbiter {
	return new(Arbiter)
}

// Creates an arbiter between the given shapes.
// If the shapes do not collide, arbiter.NumContact is zero.
func CreateArbiter(sa, sb *Shape) *Arbiter {
	arb := newArbiter()

	if sa.ShapeType() > sb.ShapeType() {
		arb.ShapeA = sb
		arb.ShapeB = sa
	} else {
		arb.ShapeA = sa
		arb.ShapeB = sb
	}

	arb.Surface_vr = vect.Vect{}

	arb.nodeA = new(ArbiterEdge)
	arb.nodeB = new(ArbiterEdge)
	arb.state =  arbiterStateFirstColl

	return arb
}

func (arb *Arbiter) destroy() {
	arb.ShapeA = nil
	arb.ShapeB = nil
	arb.NumContacts = 0
	arb.u = 0
	arb.e = 0
}

func (arb1 *Arbiter) equals(arb2 *Arbiter) bool {
	if arb1.ShapeA == arb2.ShapeA && arb1.ShapeB == arb2.ShapeB {
		return true
	}

	return false
}

func (arb *Arbiter) update(contacts *[MaxPoints]*Contact, numContacts int) {
	oldContacts := arb.Contacts
	oldNumContacts := arb.NumContacts

	sa := arb.ShapeA
	sb := arb.ShapeB

	for i := 0; i < oldNumContacts; i++ {
		oldC := oldContacts[i]
		for j := 0; j < numContacts; j++ {
			newC := contacts[j]

			if newC.hash == oldC.hash {
				newC.jnAcc = oldC.jnAcc
				newC.jtAcc = oldC.jtAcc
				newC.jBias = oldC.jBias
			}
		}
	}

	arb.Contacts = contacts
	arb.NumContacts = numContacts

	arb.u = sa.u * sb.u
	arb.e = sa.e * sb.e

	arb.Surface_vr = vect.Sub(sa.Surface_v, sb.Surface_v)
}

func (arb *Arbiter) preStep(inv_dt, slop, bias vect.Float) {

	a := arb.ShapeA.Body
	b := arb.ShapeB.Body

	for i := 0; i < arb.NumContacts; i++ {
		con := arb.Contacts[i]

		// Calculate the offsets.
		con.r1 = vect.Sub(con.p, a.p)
		con.r2 = vect.Sub(con.p, b.p)

		//con.Normal = vect.Vect{-1,0}
		

		// Calculate the mass normal and mass tangent.
		con.nMass = 1.0 / k_scalar(a, b, con.r1, con.r2, con.n)
		con.tMass = 1.0 / k_scalar(a, b, con.r1, con.r2, vect.Perp(con.n))

		// Calculate the target bias velocity.
		con.bias = -bias * inv_dt * vect.FMin(0.0, con.dist+slop)
		con.jBias = 0.0
		//con.jtAcc = 0
		//con.jnAcc = 0
		//fmt.Println("con.dist", con.dist)
		
		// Calculate the target bounce velocity.
		con.bounce = normal_relative_velocity(a, b, con.r1, con.r2, con.n) * arb.e
		//con.bounce = 0
	}
}

func (arb *Arbiter) applyCachedImpulse(dt_coef vect.Float) {
	if arb.state == arbiterStateFirstColl && arb.NumContacts > 0 {
		return
	}
	//println("asd")
	a := arb.ShapeA.Body
	b := arb.ShapeB.Body
	for i := 0; i < arb.NumContacts; i++ {
		con := arb.Contacts[i]
		j := transform.RotateVect(con.n, transform.Rotation{con.jnAcc, con.jtAcc})
		apply_impulses(a, b, con.r1, con.r2, vect.Mult(j, dt_coef))
	}
} 

func (arb *Arbiter) applyImpulse() {
	a := arb.ShapeA.Body
	b := arb.ShapeB.Body   

	for i := 0; i < arb.NumContacts; i++ {
		con := arb.Contacts[i]
		n := con.n
		r1 := con.r1
		r2 := con.r2

	
		vbn := ((((-r2.Y*b.w_bias)+b.v_bias.X)-((-r1.Y*a.w_bias)+a.v_bias.X))*n.X) + ((((r2.X*b.w_bias)+b.v_bias.Y)-((r1.X*a.w_bias)+a.v_bias.Y))*n.Y)

		// Calculate the relative velocity.
		vr := vect.Vect{(-r2.Y*b.w+b.v.X)-(-r1.Y*a.w+a.v.X), (r2.X*b.w+b.v.Y)-(r1.X*a.w+a.v.Y)}
		vrn := (vr.X*n.X) + (vr.Y*n.Y)
		
		// Calculate the relative tangent velocity.
		vrt := ((vr.X+arb.Surface_vr.X)*-n.Y) + ((vr.Y+arb.Surface_vr.Y)*n.X)

		// Calculate and clamp the bias impulse.
		jbn := (con.bias - vbn) * con.nMass
		jbnOld := con.jBias
		con.jBias = jbnOld+jbn
		if 0 > con.jBias {
			con.jBias = 0
		}
		
		// Calculate and clamp the normal impulse.
		jn := -(con.bounce + vrn) * con.nMass
		jnOld := con.jnAcc
		con.jnAcc = jnOld+jn
		if 0 > con.jnAcc {
			con.jnAcc = 0
		}


		// Calculate and clamp the friction impulse.
		jtMax := arb.u * con.jnAcc
		jt := -vrt * con.tMass
		jtOld := con.jtAcc
		con.jtAcc = jtOld+jt
		if con.jtAcc > jtMax {
			con.jtAcc = jtMax
		} else if con.jtAcc < -jtMax {
			con.jtAcc = -jtMax
		}
 

		jj := (con.jBias-jbnOld)
		j := vect.Vect{n.X*jj, n.Y*jj}
		
		a.v_bias = vect.Vect{(-j.X*a.m_inv)+a.v_bias.X, (-j.Y*a.m_inv)+a.v_bias.Y}
		a.w_bias += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))

		b.v_bias = vect.Vect{(j.X*b.m_inv)+b.v_bias.X, (j.Y*b.m_inv)+b.v_bias.Y}
		b.w_bias += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))


		jc := vect.Vect{con.jnAcc - jnOld, con.jtAcc - jtOld}

		j = vect.Vect{		
		X: (n.X*jc.X) - (n.Y*jc.Y),
		Y: (n.X*jc.Y) + (n.Y*jc.X),
		}
		 
		a.v = vect.Vect{(-j.X*a.m_inv)+a.v.X, (-j.Y*a.m_inv)+a.v.Y}
		a.w += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))
		
		b.v = vect.Vect{(j.X*b.m_inv)+b.v.X, (j.Y*b.m_inv)+b.v.Y}
		b.w += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))
	}
}


func (arb *Arbiter) applyImpulse2() {
	a := arb.ShapeA.Body
	b := arb.ShapeB.Body

	for i := 0; i < arb.NumContacts; i++ {
		con := arb.Contacts[i]
		n := con.n
		r1 := con.r1
		r2 := con.r2

		// Calculate the relative bias velocities.
		vb1 := vect.Add(a.v_bias, vect.Mult(vect.Perp(r1), a.w_bias))
		vb2 := vect.Add(b.v_bias, vect.Mult(vect.Perp(r2), b.w_bias))
		vbn := vect.Dot(vect.Sub(vb2, vb1), n)

		

		// Calculate the relative velocity.
		vr := relative_velocity(a, b, r1, r2)
		vrn := vect.Dot(vr, n)
		// Calculate the relative tangent velocity.
		vrt := vect.Dot(vect.Add(vr, arb.Surface_vr), vect.Perp(n))

		// Calculate and clamp the bias impulse.
		jbn := (con.bias - vbn) * con.nMass
		jbnOld := con.jBias
		con.jBias = vect.FMax(jbnOld+jbn, 0.0)
		
		
		// Calculate and clamp the normal impulse.
		jn := -(con.bounce + vrn) * con.nMass
		jnOld := con.jnAcc
		con.jnAcc = vect.FMax(jnOld+jn, 0.0)


		// Calculate and clamp the friction impulse.
		jtMax := arb.u * con.jnAcc
		jt := -vrt * con.tMass
		jtOld := con.jtAcc
		con.jtAcc = vect.FClamp(jtOld+jt, -jtMax, jtMax)


		// Apply the bias impulse.
		apply_bias_impulses(a, b, r1, r2, vect.Mult(n, con.jBias-jbnOld))

		// Apply the final impulse.
		apply_impulses(a, b, r1, r2, transform.RotateVect(n, transform.Rotation{con.jnAcc - jnOld, con.jtAcc - jtOld}))
		 
	}
}

