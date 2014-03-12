
////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/

function quaternion_from_axisangle( angle, axis )
{
	var unit_axis = vector_normalize(axis);
	var outq = new Array(4);
	var sine_div2 = Math.sin(angle/2);
	outq[0] = Math.cos(angle/2);
	outq[1] = sine_div2*unit_axis[0];
	outq[2] = sine_div2*unit_axis[1];
	outq[3] = sine_div2*unit_axis[2];

	if( outq[0] && outq[1] && outq[2] && outq[3] )
		return quaternion_normalize(outq);

	return outq;	
}

function quaternion_normalize( q )
{
	outq = new Array(4);
	length =  Math.sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );
	outq[0] = q[0]/length;
	outq[1] = q[1]/length;
	outq[2] = q[2]/length;
	outq[3] = q[3]/length;

	return outq;
}

function quaternion_multiply( q1, q2 )
{
	var outq = new Array(4);
	outq[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] ;
	outq[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2] ;
	outq[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1] ;
	outq[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0] ;
	return outq;
}
/*
function quaternion_to_rotation_matrix( q )
{

	var outmat = generate_identity(4);
	outmat[0][0] = q[0]^2 + q[1]^2 - q[2]^2 - q[3]^2;
	outmat[0][1] = 2*(q[1] * q[2]-q[0] * q[3]);
	outmat[0][2] = 2*(q[0] * q[2]+q[1] * q[3]);
	outmat[1][0] = 2*(q[1] * q[2] + q[0] * q[3]);
	outmat[1][1] = q[0]^2 - q[1]^2 + q[2]^2 - q[3]^2;
	outmat[1][2] = 2*(q[2] * q[3] - q[0] * q[1]);
	outmat[2][0] = 2*(q[1] * q[3] - q[0] * q[2]);
	outmat[2][1] = 2*(q[0] * q[1] + q[2] * q[3]);;
	outmat[2][2] = q[0]^2 - q[1]^2 - q[2]^2 + q[3]^2;
	outmat[3][3] = 1;	
	return outmat;

}
*/
function quaternion_to_rotation_matrix( q )
{

	var outmat = generate_identity(4);
	outmat[0][0] = 1-2*(q[2]*q[2] + q[3]*q[3]);
	outmat[0][1] = 2*(q[1]*q[2] - q[0]*q[3]);
	outmat[0][2] = 2*(q[0]*q[2] + q[1]*q[3]);

	outmat[1][0] = 2*(q[1]*q[2] + q[0]*q[3]);
	outmat[1][1] = 1-2*(q[1]*q[1] + q[3]*q[3]);
	outmat[1][2] = 2*(q[2]*q[3] - q[0]*q[1]);

	outmat[2][0] = 2*(q[1]*q[3] - q[0]*q[2]);
	outmat[2][1] = 2*(q[0]*q[1] + q[2]*q[3]);
	outmat[2][2] = 1-2*(q[1]*q[1] + q[2]*q[2]);
	
	outmat[3][3] = 1;	
	return outmat;

}

