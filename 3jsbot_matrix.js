//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/

function matrix_multiply( mat1, mat2 )
{
	var mat1cols = mat1[0].length;
	var mat1rows = mat1.length;
	var mat2cols = mat2[0].length;
	var mat2rows = mat2.length;
	var outmat = init_matrix(mat1rows, mat2cols);
	var accum = 0;
	
	if( mat1cols != mat2rows )
	{
		console.log("matrix multiply error\n");
	}
	
	for( var r = 0; r < mat1rows; r++ )
	{
		for( var c = 0; c < mat2cols; c++ )
		{
			accum = 0;
			for( var sop = 0; sop < mat1rows; sop++)
			{
				accum += mat1[r][sop] * mat2[sop][c];
			}
			outmat[r][c] = accum;
		}
	}
	return outmat;
}

function matrix_transpose( mat )
{
	var rows = mat.length;
	var cols = mat[0].length;
	var outmat = init_matrix(cols, rows);
	
	for( var r = 0; r < rows; r++ )
	{
		for( var c = 0; c < cols; c++ )
		{
			outmat[c][r] = mat[r][c];
		}
	}
	
	return outmat;
}

function vector_normalize( v )
{
	var sum = 0;
	for( var i = 0; i < v.length; i++)
		sum += v[i]*v[i];

	var length = Math.sqrt(sum);
	
	for( i = 0; i < v.length; i++ )
		v[i] = v[i]/length;

	return v;
}

function vector_cross(v1, v2)
{
	if( v1.length != 3 || v2.length != 3 )
		console.log("incorrect vector dimension");

	var outv;
	outv[0] = v1[1]*v2[2] - v1[2]*v2[1];
	outv[1] = v1[0]*v2[2] - v1[2]*v2[0];
	outv[2] = v1[0]*v2[1] - v1[1]*v2[0];

	return outv;
}

function generate_identity(dim)
{
	var outmat = init_matrix(dim, dim);
	for(var r = 0; r < dim; r++)
	{
		for(var c = 0; c < dim; c++)
		{
			if(r == c)
				outmat[r][c] = 1;
			else
				outmat[r][c] = 0;
		}
	}
	
	return outmat;
				
}

function init_matrix(rows, cols)
{
	outmat = new Array(rows);
	for(var r = 0; r < rows; r++)
		outmat[r] = new Array(cols);

	return outmat;
}

function generate_translation_matrix( x, y, z )
{
	var outmat = generate_identity(4);
	outmat[0][3] = x;
	outmat[1][3] = y;
	outmat[2][3] = z;
	return outmat;
}

function generate_rotation_matrix( x, y, z )
{
	return matrix_multiply( matrix_multiply( generate_x_rotation(x), generate_y_rotation(y) ), generate_z_rotation(z) );
}

function generate_x_rotation( angle )
{
	var outmat = generate_identity(4);
	outmat[1][1] = Math.cos(angle);
	outmat[1][2] = -Math.sin(angle);
	outmat[2][2] = Math.cos(angle);
	outmat[2][1] = Math.sin(angle);
	
	return outmat;
}

function generate_y_rotation( angle )
{
	var outmat = generate_identity(4);
	outmat[0][0] = Math.cos(angle);
	outmat[0][2] = Math.sin(angle);
	outmat[2][2] = Math.cos(angle);
	outmat[2][0] = -Math.sin(angle);
	
	return outmat;

}

function generate_z_rotation( angle )
{
	var outmat = generate_identity(4);
	outmat[0][0] = Math.cos(angle);
	outmat[0][1] = -Math.sin(angle);
	outmat[1][1] = Math.cos(angle);
	outmat[1][0] = Math.sin(angle);
	
	return outmat;


}

