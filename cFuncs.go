package chipmunk

/*
#cgo CFLAGS: -O3 -std=gnu99 -ffast-math
#cgo LDFLAGS: -O3 -std=gnu99 -ffast-math

typedef struct  {
	float X,Y; 
} vect2;

typedef struct {
	vect2 p, n ;
	float dist ;

	vect2 r1, r2;
	float nMass, tMass, bounce;

	float jnAcc, jtAcc, jBias ;
	float bias    ;            

	int hash ;
} Contact;

typedef struct {

	float m;

	float m_inv; 


	float i;
	/// Moment of inertia inverse.
	float i_inv ;

	/// Position of the rigid body's center of gravity.
	vect2 p ;
	/// Velocity of the rigid body's center of gravity.
	vect2 v ;
	/// Force acting on the rigid body's center of gravity.
	vect2 f ;

	//Transform Transform

	/// Rotation of the body around it's center of gravity in radians.
	/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
	float a ;
	/// Angular velocity of the body around it's center of gravity in radians/second.
	float w ;
	/// Torque applied to the body around it's center of gravity.
	float t ;

	/// Cached unit length vector representing the angle of the body.
	/// Used for fast rotations using cpvrotate().
	vect2 rot ;

	vect2 v_bias ;
	float w_bias ;

}Body;

inline
void Impulse(Body *a, Body *b, Contact* con, float surfx, float surfy, float u) {
		vect2 r1 = con->r1;
		vect2 r2 = con->r2;
		vect2 n = con->n;	
		float vbn = ((((-r2.Y*b->w_bias)+b->v_bias.X)-((-r1.Y*a->w_bias)+a->v_bias.X))*n.X) + 
					((((r2.X*b->w_bias)+b->v_bias.Y)-((r1.X*a->w_bias)+a->v_bias.Y))*n.Y);

		vect2 vr = {(-r2.Y*b->w+b->v.X)-(-r1.Y*a->w+a->v.X), (r2.X*b->w+b->v.Y)-(r1.X*a->w+a->v.Y)};
		float vrn = (vr.X*n.X) + (vr.Y*n.Y);	

		float vrt = ((vr.X+surfx)*-n.Y) + ((vr.Y+surfy)*n.X);

		float jbn = (con->bias - vbn) * con->nMass;
		float jbnOld = con->jBias;
		con->jBias = jbnOld+jbn;
		if (0 > con->jBias) {
			con->jBias = 0;
		}	

		float jn = -(con->bounce + vrn) * con->nMass;
		float jnOld = con->jnAcc;
		con->jnAcc = jnOld+jn;
		if (0 > con->jnAcc) {
			con->jnAcc = 0;
		}

		float jtMax = u * con->jnAcc;
		float jt = -vrt * con->tMass;
		float jtOld = con->jtAcc;
		con->jtAcc = jtOld+jt;
		if (con->jtAcc > jtMax) {
			con->jtAcc = jtMax;
		} else if (con->jtAcc < -jtMax) {
			con->jtAcc = -jtMax;
		}

		float jj = (con->jBias-jbnOld);
		vect2 j = {n.X*jj, n.Y*jj};


		a->v_bias.X = (-j.X*a->m_inv)+a->v_bias.X;
		a->v_bias.Y = (-j.Y*a->m_inv)+a->v_bias.Y;
		a->w_bias += a->i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X));

		b->v_bias.X = (j.X*b->m_inv)+b->v_bias.X;
		b->v_bias.Y = (j.Y*b->m_inv)+b->v_bias.Y;
		b->w_bias += b->i_inv * ((r2.X*j.Y) - (r2.Y*j.X));


		vect2 jc = {con->jnAcc - jnOld, con->jtAcc - jtOld};

		j.X = (n.X*jc.X) - (n.Y*jc.Y);
		j.Y = (n.X*jc.Y) + (n.Y*jc.X);


		a->v.X = (-j.X*a->m_inv)+a->v.X;
		a->v.Y = (-j.Y*a->m_inv)+a->v.Y;
		a->w += a->i_inv * ((r1.X*-j.Y) - (r1.Y*-j.X));

		b->v.X = (j.X*b->m_inv)+b->v.X;
		b->v.Y = (j.Y*b->m_inv)+b->v.Y;
		b->w += b->i_inv * ((r2.X*j.Y) - (r2.Y*j.X));
}



*/
/*
import "C"
import (
	. "github.com/vova616/chipmunk/vect"
	"unsafe"
)

func Impulse(a, b *Body, con *Contact, surf Vect, u float32) {
	C.Impulse(
		(*C.Body)(unsafe.Pointer(a)),
		(*C.Body)(unsafe.Pointer(b)),
		(*C.Contact)(unsafe.Pointer(con)),
		C.float(surf.X), C.float(surf.Y),
		(C.float)(u))

}
*/
