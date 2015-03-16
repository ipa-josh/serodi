
function Joystick(id, ros) {
	this.state = false;
	this.touchable = 'createTouch' in document;
	this.el = $("#"+id);
	this.ros = ros;
	this.enabled = true;
	
	this.resetCanvas = function(e) {  
		this.canvas.width = this.el.width(); 
		this.canvas.height = this.el.height();
	}

	/*	
	 *	Touch event (e) properties : 
	 *	e.touches: 			Array of touch objects for every finger currently touching the screen
	 *	e.targetTouches: 	Array of touch objects for every finger touching the screen that
	 *						originally touched down on the DOM object the transmitted the event.
	 *	e.changedTouches	Array of touch objects for touches that are changed for this event. 					
	 *						I'm not sure if this would ever be a list of more than one, but would 
	 *						be bad to assume. 
	 *
	 *	Touch objects : 
	 *
	 *	identifier: An identifying number, unique to each touch event
	 *	target: DOM object that broadcast the event
	 *	clientX: X coordinate of touch relative to the viewport (excludes scroll offset)
	 *	clientY: Y coordinate of touch relative to the viewport (excludes scroll offset)
	 *	screenX: Relative to the screen
	 *	screenY: Relative to the screen
	 *	pageX: Relative to the full page (includes scrolling)
	 *	pageY: Relative to the full page (includes scrolling)
	 */	
	this.onTouchStart = {
		obj: this,
		handleEvent: function(e) {
			for(var i = 0; i<e.changedTouches.length; i++){
				var touch =e.changedTouches[i];
				
				this.obj.touch_start = {x: touch.clientX-canvas.width/2, y: touch.clientY-canvas.height/2};
				this.obj.state = true;
				this.obj.drawJoystick( {start: {x: canvas.width/2, y: canvas.height/2}, x: touch.clientX-this.obj.touch_start.x, y: touch.clientY-this.obj.touch_start.y, aim: this.obj.state});
				break;
			}
			touches = e.touches;
		}
	}
	 
	this.onTouchMove = {
		obj: this,
		handleEvent: function(e) {
			 // Prevent the browser from doing its default thing (scroll, zoom)
			e.preventDefault();
			
			for(var i = 0; i<e.changedTouches.length; i++){
				var touch =e.changedTouches[i]; 
				
				this.obj.drawJoystick( {start: {x: canvas.width/2, y: canvas.height/2}, x: touch.clientX-this.obj.touch_start.x, y: touch.clientY-this.obj.touch_start.y, aim: this.obj.state});
				break;
			}
			
			touches = e.touches; 
		}
	} 
	 
	this.onTouchEnd = {
		obj: this,
		handleEvent: function(e) {
	   
			touches = e.touches; 
			for(var i = 0; i<e.changedTouches.length; i++){
				var touch =e.changedTouches[i]; 
				
				this.obj.state = false;
				this.obj.drawJoystick( {start: {x: canvas.width/2, y: canvas.height/2}, x: touch.clientX-this.obj.touch_start.x, y: touch.clientY-this.obj.touch_start.y, aim: this.obj.state});
				break;
			}
	   }
	}
	
	this.onMouseMove = {
		obj: this,
		handleEvent: function(event) {
			mouseX = event.offsetX;
			mouseY = event.offsetY;
			
			this.obj.drawJoystick( {start: {x: canvas.width/2, y: canvas.height/2}, x: mouseX, y: mouseY, aim: this.obj.state});
		}
	};
	this.onMouseDown = {
		obj: this,
		handleEvent: function(event) {
			this.obj.state = true;
			this.obj.onMouseMove.handleEvent(event);
		}
	};
	this.onMouseUp = {
		obj: this,
		handleEvent: function(event) {
			this.obj.state = false;
			this.obj.onMouseMove.handleEvent(event);
		}
	};
	
	this.drawJoystick = function(touch) {
		this.c.clearRect(0,0,canvas.width, canvas.height); 
		
		if(!this.enabled) return;
		
		this.c.beginPath(); 
		this.c.strokeStyle = "cyan"; 
		this.c.lineWidth = 6; 
		this.c.arc(touch.start.x, touch.start.y, 40,0,Math.PI*2,true); 
		this.c.stroke();
		this.c.beginPath(); 
		this.c.strokeStyle = "cyan"; 
		this.c.lineWidth = 2; 
		this.c.arc(touch.start.x, touch.start.y, 60,0,Math.PI*2,true); 
		this.c.stroke();
		
		if(touch.aim) {
			this.c.beginPath(); 
			this.c.strokeStyle = "cyan"; 
			this.c.arc(touch.x, touch.y, 40, 0,Math.PI*2, true); 
			this.c.stroke();
				
			var dx = -(touch.x-touch.start.x);
			var dy = -(touch.y-touch.start.y);
			var tdx = 0;
			var tdy = 0;
			if(dx>20) tdx = dx-20;
			else if(dx<-20) tdx = dx+20;
			if(dy>20) tdy = dy-20;
			else if(dy<-20) tdy = dy+20;

			var SPEED_LIMIT = 0.4;
			var SPEED_LIMITR = 0.4;
			var twist = new ROSLIB.Message({
				linear : {
				  x : Math.max(-SPEED_LIMIT, Math.min(SPEED_LIMIT, tdy/100.) ),
				  y : Math.max(-SPEED_LIMIT, Math.min(SPEED_LIMIT, tdx/100.) ),
				  z : 0
				},
				angular : {
				  x : 0,
				  y : 0,
				  z : Math.max(-SPEED_LIMITR, Math.min(SPEED_LIMITR, Math.atan2(dx,dy)))
				}
			});
			this.pub_cmd.publish(twist);
			//console.log(twist);
		}
	}
	
	
	var canvas = document.createElement( 'canvas' );
	this.canvas = canvas;
	this.c = canvas.getContext( '2d' );
	container = document.getElementById(id);
	//container.className = "container";
	//document.body.appendChild( container );
	container.appendChild(canvas);	
	this.resetCanvas(); 
	
	this.c.strokeStyle = "#ffffff";
	this.c.lineWidth =2;	
	
	if(this.touchable) {
		canvas.addEventListener( 'touchstart', this.onTouchStart, false );
		canvas.addEventListener( 'touchmove', this.onTouchMove, false );
		canvas.addEventListener( 'touchend', this.onTouchEnd, false );
		//window.onorientationchange = this.resetCanvas;  
		//window.onresize = this.resetCanvas;  
	} else {
		canvas.addEventListener( 'mousedown', this.onMouseDown, false );
		canvas.addEventListener( 'mouseup', this.onMouseUp, false );
		canvas.addEventListener( 'mouseout', this.onMouseUp, false );
		canvas.addEventListener( 'mousemove', this.onMouseMove, false );
	}
	
	this.pub_cmd = new ROSLIB.Topic({
		ros : ros,
		name : '/base/twist_controller/command',
		messageType : 'geometry_msgs/Twist'
	});
	
	this.update = function() {
		this.drawJoystick( {start: {x: canvas.width/2, y: canvas.width/2}, aim: this.state});
	};
	
	this.update();
	//setInterval(draw, 1000/35);
}
