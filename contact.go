package chipmunk

import (
	. "github.com/vova616/chipmunk/vect"
)
 
type HashValue uint32

type Contact struct {
	p, n Vect
	dist Float

	r1, r2               Vect
	nMass, tMass, bounce Float

	jnAcc, jtAcc, jBias Float
	bias                Float

	hash HashValue
}

func (con *Contact) reset(pos, norm Vect, dist Float, hash HashValue) {
	con.p = pos
	con.n = norm
	con.dist = dist
	con.hash = hash

	con.jnAcc = 0.0
	con.jtAcc = 0.0
	con.jBias = 0.0
}

func (con *Contact) Normal() Vect {
	return con.n
}