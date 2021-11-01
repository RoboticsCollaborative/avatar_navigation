var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.')
});


ros.on('error', function (error) {

    console.log('Error connecting to websocket server: ', error);

});

ros.on('close', function () {

    console.log('Connection to websocket server closed.');

});

reach_clear = true;

// Subscribers
//system state subscriber (topic)
var weightSub = new ROSLIB.Topic({
    ros: ros,
    name: '/load_mass',
    messageType: 'std_msgs/Float64MultiArray'
})

var homeSub = new ROSLIB.Topic({
    ros: ros,
    name: '/is_homing',
    messageType: 'std_msgs/Bool'
})

var jointSub = new ROSLIB.Topic({
    ros: ros,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
})

jointSub.subscribe(function (message) {
    elbow = message.position[0];
    lift = message.position[1];
    if (Math.abs(elbow) < 0.62 && Math.abs(lift) < 0.5) {
        document.getElementById("prompt_limit").innerText = "Reaching limit!";
        reach_clear = false;
    }
    else if (!reach_clear){
        document.getElementById("prompt_limit").innerText = "";
        reach_clear = true;
    }
})

weightSub.subscribe(function (message) {
    weight = -message.data[2];
    document.getElementById("weight_value").innerText = weight.toString().slice(0,4) + "kg";
})

homeSub.subscribe(function (message) {
    homing = message.data;
    if (homing == true) {
        document.getElementById("info_div").style.background = "#c20922";
        document.getElementById("weight_value").style.visibility = "hidden";
        document.getElementById("weight_text").style.visibility = "hidden";
        document.getElementById("prompt_homing").innerText = "PLEASE HOLD STILL!!"
    }
    else {
        document.getElementById("info_div").style.background = "#20c209";
        document.getElementById("weight_value").style.visibility = "visible";
        document.getElementById("weight_text").style.visibility = "visible";
        document.getElementById("prompt_homing").innerText = ""

    }
})
