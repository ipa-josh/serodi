
function Map(src, parent, ros) {
	this.src = src;
	this.parent = parent;
	this.ros = ros;
	this.map_info;
	this.poses = [];
	this.pose_var = [0,0];
	this.active_pose = false;
	this.touchable = 'createTouch' in document;
	
	this.settings = {'add_enabled': false};
	this.on_updated_pose = function() {};
	
	this.canvas = document.createElement( 'canvas' );
	this.c = this.canvas.getContext( '2d' );
	this.parent.append(this.canvas);
	
	this.c.strokeStyle = "#ffffff";
	this.c.lineWidth =2;	
	
	this.icon={};
	this.icon['cut']=new Image();
	this.icon['cut'].src="img/cut.png";
	this.icon['update']=new Image();
	this.icon['update'].src="img/update.png";
	this.icon['add']=new Image();
	this.icon['add'].src="img/add.png";
	
	this.img=new Image();
	var c = this.c;
	var canvas = this.canvas;
	this.img.addEventListener("load", function() {
		canvas.width = this.width; 
		canvas.height = this.height;
	}, false);
	
	var RADIUS = 15;
	this.onMouseDown = {
		obj: this,
		handleEvent: function(event) {
			var x=event.offsetX;
			var y=event.offsetY;
			if(this.obj.settings['add_enabled'] && this.obj.robot_pose &&
				x>=4 && x<=this.obj.icon['add'].width+4 &&
				y>=4 && y<=this.obj.icon['add'].height+4 ) {
				this.obj.addPose(this.obj.robot_pose[0],this.obj.robot_pose[1],this.obj.robot_pose[2], "");
				return;
			}
			
			if(this.obj.active_pose) {
				if(	x>=58 && x<=this.obj.icon['cut'].width+58 &&
					y>=4 && y<=this.obj.icon['cut'].height+4 ) {
					this.obj.poses.splice(this.obj.poses.indexOf(this.obj.active_pose), 1);
					this.obj.active_pose=false;
					this.obj.on_updated_pose(this.obj.poses);
					return;
				}
				
				if(	x>=112 && x<=this.obj.icon['update'].width+112 &&
					y>=4 && y<=this.obj.icon['update'].height+4 ) {
					this.obj.active_pose[0] = this.obj.robot_pose[0];
					this.obj.active_pose[1] = this.obj.robot_pose[1];
					this.obj.active_pose[2] = this.obj.robot_pose[2];
					this.obj.on_updated_pose(this.obj.poses);
					return;
				}
			}
			
			this.obj.active_pose=false;
			
			if(!this.map_info) return;
			
			for(i in this.obj.poses) {
				var p=this.obj.poses[i];
				var pos=this.obj.pose2img(p);
				if( (pos.x-x)*(pos.x-x) + (pos.y-y)*(pos.y-y) <= RADIUS*RADIUS) {
					this.obj.active_pose = p;
					break;
				}
			}
		}
	};
		
	
	this.canvas.addEventListener( 'mousedown', this.onMouseDown, false );
	
	this.pose2img = function(p) {
			var r = this.map_info.resolution;
			var ox = this.map_info.origin.position.x;
			var oy = this.map_info.origin.position.y;
			
			return {x: (p[0]-ox)/r, y: this.map_info.height-1-(p[1]-oy)/r, yaw: p[2]};
	}
	
	this.draw = function() {		
		this.c.drawImage(this.img,0,0);
			
		for(i in this.poses) {
			if(!this.map_info) continue;
			
			var p=this.poses[i];
			var pos = this.pose2img(p);
			
			this.c.beginPath(); 
			this.c.strokeStyle = "#2C52B0"; 
			if(p==this.active_pose)
				this.c.strokeStyle = "#F99830"; 
			this.c.lineWidth = RADIUS; 
			this.c.arc( pos.x, pos.y, 15,pos.yaw-0.2,pos.yaw+0.2,true);
			this.c.stroke();
			
			this.c.beginPath(); 
			this.c.fillStyle = "black";
			this.c.font="14px Verdana";
			this.c.fillText(p[3], pos.x-6,pos.y+5);
		}
		
		if(this.robot_pose && this.map_info) {
			var p = this.robot_pose;
			var pos = this.pose2img(p);

			//draw var. trans.
			/*var grd=this.c.createRadialGradient(pos.x, pos.y, 1, pos.x, pos.y, this.pose_var[0]/this.map_info.resolution);
			grd.addColorStop(0,"red");
			grd.addColorStop(1,"transparent");
			
			this.c.beginPath(); 
			this.c.fillStyle = grd;
			this.c.lineWidth = 0;
			var a1 = pos.yaw-this.pose_var[1], a2 = pos.yaw+this.pose_var[1];
			if(a1<0) a1+=2+Math.pi;
			if(a2<0) a2+=2+Math.pi;
			console.log(Math.min(a1,a2),Math.max(a1,a2));
			this.c.arc( pos.x, pos.y, this.pose_var[0]/this.map_info.resolution,Math.min(a1,a2),Math.max(a1,a2),true); 
			this.c.fill();

			//draw var. rot.
			var grd=this.c.createRadialGradient(pos.x, pos.y, 1, pos.x, pos.y, this.pose_var[1]/this.map_info.resolution);
			grd.addColorStop(0,"red");
			grd.addColorStop(1,"transparent");
			
			this.c.beginPath(); 
			this.c.fillStyle = grd;
			this.c.lineWidth = 0;
			this.c.arc( pos.x, pos.y, this.pose_var/this.map_info.resolution,pos.yaw-this.pose_var[1],pos.yaw+this.pose_var[1],true); 
			this.c.fill();*/
		
			//draw robot
			if(this.pose_var[0]>0 && this.pose_var[1]>0) {
				var a1 = pos.yaw-this.pose_var[1], a2 = pos.yaw+this.pose_var[1];
				if(a1<0) a1+=2*Math.PI;
				if(a2<0) a2+=2*Math.PI;
				
				this.c.beginPath(); 
				this.c.strokeStyle = "#75A16E"; 
				this.c.lineWidth = RADIUS/5;
				this.c.arc( pos.x, pos.y, 15,pos.yaw-0.2,pos.yaw+0.2,true); 
				this.c.stroke();
				
				this.c.beginPath(); 
				this.c.strokeStyle = "#43D729"; 
				this.c.lineWidth = RADIUS/3;
				this.c.arc( pos.x, pos.y, this.pose_var[0]/this.map_info.resolution,Math.min(a1,a2),Math.max(a1,a2) ,true); 
				this.c.stroke();
			}
			else {
				this.c.beginPath(); 
				this.c.strokeStyle = "#43D729"; 
				this.c.lineWidth = RADIUS/3;
				this.c.arc( pos.x, pos.y, 15,pos.yaw-0.2,pos.yaw+0.2,true); 
				this.c.stroke();
			}
		}
		
		if(this.robot_pose) {
			if(this.settings['add_enabled'])
				this.c.drawImage(this.icon['add'],4,4);
		}
		if(this.active_pose) {
			this.c.drawImage(this.icon['cut'],58,4);
			this.c.drawImage(this.icon['update'],112,4);
		}
	}
	this.update = function() {
		var d = new Date();
		this.img.src = this.src+"?"+d.getTime();
		this.draw();
	}
	this.clearPoses = function() {
		this.poses = [];
		this.active_pose=false;
		this.on_updated_pose(this.poses);
	}
	this.addPose = function(x,y,yaw,txt) {
		var ind = this.poses.length-1;
		if(this.active_pose)
			this.poses.splice(this.poses.indexOf(this.active_pose),0,[x,y,yaw,txt]);
		else
			this.poses.push([x,y,yaw,txt]);
		this.on_updated_pose(this.poses);
		return this.poses[ind];
	}
	this.destroy = function() {
		clearInterval(this.refreshIntervalId);
		this.sub_map_info.unsubscribe();
		this.sub_localized.unsubscribe();
		this.tfClient.unsubscribe('/base_link');
		this.parent.remove();
	}
	
	var self=this;
	
	// Like when publishing a topic, we first create a Topic object with details of the topic's name
	// and message type. Note that we can call publish or subscribe on the same topic object.
	var sub_map_info = new ROSLIB.Topic({
		ros : ros,
		name : '/map_update',
		messageType : 'nav_msgs/MapMetaData'
	});
	// Then we add a callback to be called every time a message is published on this topic.
	sub_map_info.subscribe(function(message) {
		self.map_info = message;
		self.update();
		console.log("map_update ",self.map_info);
	});
	this.sub_map_info = sub_map_info;
	
	var sub_localized = new ROSLIB.Topic({
		ros : ros,
		name : '/ui/localized',
		messageType : 'geometry_msgs/PoseWithCovarianceStamped'
	});
	// Then we add a callback to be called every time a message is published on this topic.
	sub_localized.subscribe(function(pwcs) {
		self.pose_var =  [pwcs.pose.covariance[0], pwcs.pose.covariance[pwcs.pose.covariance.length-1]]
		//self.update();
		console.log("localized ",self.pose_var);
	});
	this.sub_localized = sub_localized;
	
	//get map info
	var sub_map = new ROSLIB.Topic({
		ros : ros,
		name : '/map',
		messageType : 'nav_msgs/OccupancyGrid'
	});
	sub_map.subscribe(function(pwcs) {
		sub_map.unsubscribe();
	});
	
	// TF Client
	// ---------
	var tfClient = new ROSLIB.TFClient({
		ros : ros,
		fixedFrame : '/map',
		angularThres : 0.01,
		transThres : 0.01
	});
	// Subscribe to a turtle.
	tfClient.subscribe('/base_link', function(tf) {
		var yaw = Math.atan2( 2*(tf.rotation.z*tf.rotation.w), 1-2*tf.rotation.z*tf.rotation.z );
		self.robot_pose = [tf.translation.x, tf.translation.y, -yaw];
		console.log(tf);
	});
	this.tfClient = tfClient;
	
	this.update();
	this.refreshIntervalId = setInterval(function() {self.draw();}, 1000/200);
}
