
function start_logging(ros, tbl) {
	var table = tbl.DataTable();
	
  var sub = new ROSLIB.Topic({
    ros : ros,
    name : '/ui/log',
    messageType : 'std_msgs/String'
  });
  // Then we add a callback to be called every time a message is published on this topic.
  sub.subscribe(function(message) {
    console.log("ui log: ",message);
    //sub_menu.unsubscribe();
    
    var arr = message.data.split(';'), result = arr.splice(0,2);
	result.push(arr.join(';'));
    if(result.length<3) return;
    
    result[0] = result[0].split("/")
    result[0] = result[0][result[0].length-1].split(".")[0]
    
    table.row.add(result).draw();
  });
  
}
