//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () 
{
	
	var curdate = new Date();
	console.log(curdate.getSeconds()/60);
	for(var x in robot.joints )
	{
		robot.joints[x].servo.p_desired = (curdate.getSeconds() / 60)*2*Math.PI;
		robot.joints[x].control = .1 * (robot.joints[x].servo.p_desired - robot.joints[x].angle); 
	}
}

