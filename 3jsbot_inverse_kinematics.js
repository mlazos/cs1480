//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {

		var link_xform = robot.joints[endeffector_joint].xform;
		var translation = generate_translation_matrix( endeffector_local_pos[0][0], endeffector_local_pos[1][0], endeffector_local_pos[2][0] );
		var end_xform = matrix_multiply( link_xform, translation );
		
		
		var end_vec = matrix_multiply( link_xform,[ [ endeffector_local_pos[0][0] ], [ endeffector_local_pos[1][0] ], [ endeffector_local_pos[2][0] ], [1] ] );
		var target_translation = generate_translation_matrix( target_pos[0][0], target_pos[1][0], target_pos[2][0] );
        iterate_inverse_kinematics(target_translation, endeffector_joint, end_vec);
		
		var endeffector_mat = matrix_2Darray_to_threejs( end_xform );
		simpleApplyMatrix( endeffector_geom, endeffector_mat );
		endeffector_geom.visible = true;
        
		target_geom.visible = true;
		var target_mat = matrix_2Darray_to_threejs(  target_translation );
		simpleApplyMatrix( target_geom, target_mat );		
		
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}

function iterate_inverse_kinematics( target_xform, endeffector_joint, end_xform)
{
	var delta_x = [[target_xform[0][3] - end_xform[0][0]],
				   [target_xform[1][3] - end_xform[1][0]],
				   [target_xform[2][3] - end_xform[2][0]],
					[0],[0],[0] ];
	var alpha = 0.1;
	var joint_chain_start = [];
	var joint_chain = gen_kinematic_chain_joint( endeffector_joint, joint_chain_start );
	jacobian = init_matrix( 6, joint_chain.length );
	On = new Array(3);
	
	var base_xform = robot.links["base"].xform;
	On[0] = end_xform[0][0];
	On[1] = end_xform[1][0];
	On[2] = end_xform[2][0];
	
	for( var i = 0; i < joint_chain.length; i++ )
	{
		jacobian = fill_jacobian_col(i, jacobian, joint_chain[i], On);
	}
		
	//if( typeof watch_jacobian === 'undefined' )
	//	watch_jacobian = jacobian;

	console.log(jacobian);
	var pseudo = 1; 
	if( pseudo )
	{
		
		var matrix_to_invert = matrix_multiply(jacobian, matrix_transpose(jacobian) );
		var matrix_to_invert2 = matrix_multiply(matrix_transpose(jacobian),jacobian );
		console.log("matrix_to_invert2");
		console.log(matrix_to_invert2);
		var inverted_jacobian_transpose = numeric.inv( matrix_to_invert2);	
		console.log(inverted_jacobian_transpose);
		var pseudo_inverse = matrix_multiply(  inverted_jacobian_transpose, matrix_transpose(jacobian));
		console.log("pseudo_inverse");
		console.log(pseudo_inverse);
		var delta_theta = matrix_multiply(pseudo_inverse , delta_x );	
	}
	else
	{
		var delta_theta = numeric.dot( matrix_transpose(jacobian), delta_x );
	}
	
	for( var i = 0; i < joint_chain.length; i++ )
	{
		robot.joints[joint_chain[i]].control = alpha * delta_theta[i][0];
	}
}

function gen_kinematic_chain_joint( joint, joint_chain  )
{
	joint_chain.push(joint);
	return gen_kinematic_chain_link( robot.joints[joint].parent, joint_chain );
}

function gen_kinematic_chain_link( link, joint_chain  )
{
	if( link == "base" )
	{
		return joint_chain;
	}
	else
	{ 
		return gen_kinematic_chain_joint( robot.links[link].parent, joint_chain );
	}
}

function fill_jacobian_col( col, jacobian, joint, On  )
{
	var joint_vector = new Array(3);
	joint_vector[0] = robot.joints[joint].xform[0][3];
	joint_vector[1] = robot.joints[joint].xform[1][3];
	joint_vector[2] = robot.joints[joint].xform[2][3];
	
	//this is the difference between On and the vector from the base to the joint
	var diff_xform  = [[On[0] - joint_vector[0]],
					 [On[1] - joint_vector[1]],
					 [On[2] - joint_vector[2]],
						[1]];
    
	var axis = robot.joints[joint].axis;
	var joint_xform = robot.joints[joint].xform;
	joint_xform[0][3] = 0;
	joint_xform[1][3] = 0;
	joint_xform[2][3] = 0;
	joint_xform[3][3] = 1;
	
	axis_xform = matrix_multiply( joint_xform, [[axis[0]],[axis[1]],[axis[2]],[1]]);
		
	var axis_v = new Array(3);
	axis_v[0]  = axis_xform[0][0];
	axis_v[1]  = axis_xform[1][0];
	axis_v[2]  = axis_xform[2][0];

	diff_v = new Array(3);
	diff_v[0] = diff_xform[0][0];
	diff_v[1] = diff_xform[1][0];
	diff_v[2] = diff_xform[2][0];
	
	var j_col = vector_cross( axis_v, diff_v );
	jacobian[0][col] = j_col[0];
	jacobian[1][col] = j_col[1];
	jacobian[2][col] = j_col[2];
	jacobian[3][col] = axis_v[0];
	jacobian[4][col] = axis_v[1];
	jacobian[5][col] = axis_v[2];

	return jacobian;
}



