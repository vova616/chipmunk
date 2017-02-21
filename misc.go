package chipmunk

import (
	"github.com/Seb-C/chipmunk/vect"
	"log"
)


func k_scalar_body(body *Body, r, n vect.Vect) vect.Float {
	rcn := vect.Cross(r, n)
	return body.m_inv + (body.i_inv*rcn*rcn)
}

func k_scalar(a, b *Body, r1, r2, n vect.Vect) vect.Float {
	value := k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n)
	if value == 0.0 {
		log.Printf("Warning: Unsolvable collision or constraint.")
	}
	return value
}


func k_scalar2(a, b *Body, r1, r2, n vect.Vect) vect.Float {
	rcn := (r1.X*n.Y) - (r1.Y*n.X)
	rcn = a.m_inv + (a.i_inv*rcn*rcn)
	
	rcn2 := (r2.X*n.Y) - (r2.Y*n.X)
	rcn2 = b.m_inv + (b.i_inv*rcn2*rcn2)
	
	value := rcn + rcn2
	if value == 0.0 {
		log.Printf("Warning: Unsolvable collision or constraint.")
	}
	return value 
}

func relative_velocity2(a, b *Body, r1, r2 vect.Vect) vect.Vect {
	v1 := vect.Add(b.v, vect.Mult(vect.Perp(r2), b.w))
	v2 := vect.Add(a.v, vect.Mult(vect.Perp(r1), a.w))
	return vect.Sub(v1, v2)
}

func relative_velocity(a, b *Body, r1, r2 vect.Vect) vect.Vect {
	return vect.Vect{(-r2.Y*b.w+b.v.X)-(-r1.Y*a.w+a.v.X), (r2.X*b.w+b.v.Y)-(r1.X*a.w+a.v.Y)}
}
 
func normal_relative_velocity(a, b *Body, r1, r2, n vect.Vect) vect.Float {
	return vect.Dot(relative_velocity(a, b, r1, r2), n)
}



func apply_impulses(a, b *Body, r1, r2, j vect.Vect) {
	j1 := vect.Vect{-j.X, -j.Y}
	
	a.v.Add(vect.Mult(j1, a.m_inv))
	a.w += a.i_inv * vect.Cross(r1, j1)
	
	
	
	b.v.Add(vect.Mult(j, b.m_inv))
	b.w += b.i_inv * vect.Cross(r2, j)
}

func apply_bias_impulses(a, b *Body, r1, r2, j vect.Vect) {

	j1 := vect.Vect{-j.X, -j.Y}
	
	a.v_bias.Add(vect.Mult(j1, a.m_inv))
	a.w_bias += a.i_inv * vect.Cross(r1, j1)
	
	b.v_bias.Add(vect.Mult(j, b.m_inv))
	b.w_bias += b.i_inv * vect.Cross(r2, j)
}

/*
func apply_impulses(a, b *Body, r1, r2, j vect.Vect) {
	
	a.v = vect.Vect{(-j.X*a.m_inv)+a.v.X, (-j.Y*a.m_inv)+a.v.Y}
	a.w += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))
	
	b.v = vect.Vect{(j.X*b.m_inv)+b.v.X, (j.Y*b.m_inv)+b.v.Y}
	b.w += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))
}

func apply_bias_impulses(a, b *Body, r1, r2, j vect.Vect) {

	a.v_bias = vect.Vect{(-j.X*a.m_inv)+a.v_bias.X, (-j.Y*a.m_inv)+a.v_bias.Y}
	a.w_bias += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))

	b.v_bias = vect.Vect{(j.X*b.m_inv)+b.v_bias.X, (j.Y*b.m_inv)+b.v_bias.Y}
	b.w_bias += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))
}
<<<<<<< HEAD
=======
 
/*
func apply_impulses(a, b *Body, r1, r2, j vect.Vect) {
	
	a.v = vect.Vect{(-j.X*a.m_inv)+a.v.X, (-j.Y*a.m_inv)+a.v.Y}
	a.w += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))
	
	b.v = vect.Vect{(j.X*b.m_inv)+b.v.X, (j.Y*b.m_inv)+b.v.Y}
	b.w += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))
}

func apply_bias_impulses(a, b *Body, r1, r2, j vect.Vect) {

	a.v_bias = vect.Vect{(-j.X*a.m_inv)+a.v_bias.X, (-j.Y*a.m_inv)+a.v_bias.Y}
	a.w_bias += a.i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X))

	b.v_bias = vect.Vect{(j.X*b.m_inv)+b.v_bias.X, (j.Y*b.m_inv)+b.v_bias.Y}
	b.w_bias += b.i_inv * ((r2.X*j.Y) - (r2.Y*j.X))
}
>>>>>>> Performance improvement
*/
