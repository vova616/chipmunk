package chipmunk

import (
	"chipmunk/vect"
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

func relative_velocity(a, b *Body, r1, r2 vect.Vect) vect.Vect {
	v1_sum := vect.Add(a.v, vect.Mult(vect.Perp(r1), a.w))
	v2_sum := vect.Add(b.v, vect.Mult(vect.Perp(r2), b.w))

	return vect.Sub(v2_sum, v1_sum)
}

func normal_relative_velocity(a, b *Body, r1, r2, n vect.Vect) vect.Float {
	return vect.Dot(relative_velocity(a, b, r1, r2), n)
}

func apply_impulses(a, b *Body, r1, r2, j vect.Vect) {
	apply_impulse(a, vect.Mult(j, -1), r1)
	apply_impulse(b, j, r2)
}

func apply_impulse(body *Body, j, r vect.Vect) {
	body.v.Add(vect.Mult(j, body.m_inv))
	body.w += body.i_inv * vect.Cross(r, j)
}

func apply_bias_impulses(a, b *Body, r1, r2, j vect.Vect) {
	apply_bias_impulse(a, vect.Mult(j, -1), r1)
	apply_bias_impulse(b, j, r2)
}

func apply_bias_impulse(body *Body, j, r vect.Vect) {
	body.v_bias.Add(vect.Mult(j, body.m_inv))
	body.w_bias += body.i_inv * vect.Cross(r, j)
}
