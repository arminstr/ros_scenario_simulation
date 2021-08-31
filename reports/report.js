// ros_scenario_simulation
// Visualization of CI Framework
// Copyright 2021, Armin Straller,
// University of Applied Sciences Augsburg
// All rights reserved.
// Licensed under the MIT license

var timerId = 0;
var htmlCanvas = document.getElementById("scenarioCanvas");
var meterToPixelFactor = htmlCanvas.height / 20.0;
var zoomInButton = document.getElementById("scenarioCanvasZoomIn");
zoomInButton.addEventListener("click", (event) => {
    meterToPixelFactor *= 1.1;
});
var zoomOutButton = document.getElementById("scenarioCanvasZoomOut");
zoomOutButton.addEventListener("click", (event) => {
    meterToPixelFactor *= 0.9;
});

const fileSelector = document.getElementById('file-selector');
  fileSelector.addEventListener('change', (event) => {
    const fileList = event.target.files;
    console.log(fileList);
    clearInterval(timerId);
    readScenarioJson(fileList[0]);
});

function readScenarioJson(file)
{
    // Check if the file is json.
    if (file.type && !file.type.startsWith('application/json')) {
      console.log('File is not an json.', file.type, file);
      return;
    }
  
    const reader = new FileReader();
    reader.addEventListener('load', (event) => {
        scenario_content = JSON.parse(reader.result);
        drawCharts(scenario_content);
        startVideo(scenario_content);
        
    });
    console.log(file);
    reader.readAsText(file);
}

function startVideo(scenario){
    var i = 0;
    timerId = setInterval(function() {
        drawVideo(scenario, i);
        i++;
        if(i >= scenario["timeSteps"].length)
        {
            clearInterval(timerId);
            startVideo(scenario);
        }
    }, 100);
}

function drawCharts(scenario)
{
    var vel_div = document.getElementById('velocityChart');
    // Define Data
    var vel_data = [{
        x: scenario["timeSteps"],
        y: scenario["velocity"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var vel_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Velocity [m/s]"},
        title: "Velocity of Ego Vehicle [m/s]"
    };

    // Display using Plotly
    Plotly.newPlot(vel_div, vel_data, vel_layout);

    var accel_div = document.getElementById('accelerationChart');
    // Define Data
    var accel_data = [{
        x: scenario["timeSteps"],
        y: scenario["acceleration"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var accel_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Acceleration [m/s^2]"},
        title: "Acceleration of Ego Vehicle [m/s^2]"
    };

    // Display using Plotly
    Plotly.newPlot(accel_div, accel_data, accel_layout);

    var jerk_div = document.getElementById('jerkChart');
    // Define Data
    var jerk_data = [{
        x: scenario["timeSteps"],
        y: scenario["jerk"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var jerk_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Jerk [m/s^3]"},
        title: "Jerk of Ego Vehicle [m/s^3]"
    };

    // Display using Plotly
    Plotly.newPlot(jerk_div, jerk_data, jerk_layout);

    var steer_div = document.getElementById('steeringChart');
    // Define Data
    var steer_data = [{
        x: scenario["timeSteps"],
        y: scenario["steeringAngle"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var steer_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Steering Angle [deg]"},
        title: "Steering Angle of Ego Vehicle [deg]"
    };

    // Display using Plotly
    Plotly.newPlot(steer_div, steer_data, steer_layout);

    var steering_rate_div = document.getElementById('steeringRateChart');
    // Define Data
    var steering_rate_data = [{
        x: scenario["timeSteps"],
        y: scenario["steeringRate"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var steering_rate_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Steering Rate [deg/s]"},
        title: "Steering Rate of Ego Vehicle [deg/s]"
    };

    // Display using Plotly
    Plotly.newPlot(steering_rate_div, steering_rate_data, steering_rate_layout);

    var yaw_rate_div = document.getElementById('yawRateChart');
    // Define Data
    var yaw_rate_data = [{
        x: scenario["timeSteps"],
        y: scenario["yawRate"],
        mode: "lines",
        type: "scatter"
    }];

    // Define Layout
    var yaw_rate_layout = {
        xaxis: {title: "Time [s]"},
        yaxis: {title: "Yaw Rate [rad/s]"},
        title: "Yaw Rate of Ego Vehicle [rad/s]"
    };

    // Display using Plotly
    Plotly.newPlot(yaw_rate_div, yaw_rate_data, yaw_rate_layout);
}

function drawVideo(scenario, timeStep)
{
    var ctx = htmlCanvas.getContext("2d");
    var egoColor = "#00FF00";
    var egoLineWidth = 2.5;
    var egoLength = 5.0;
    var egoWidth = 2.5;
    var obstacleColor = "#FF0000";
    var obstacleLineWidth = 1.0;
    var egoTraceLength = 50;
    var pastEgoColor = "#6495ED";
    var bTraces = true;
    var mapLineColor = "#AAAAAA";
    var mapLineWidth = 1.0;

    // clear the canvas
    ctx.beginPath();
    ctx.clearRect(0, 0, htmlCanvas.width, htmlCanvas.height);

    draw_map(htmlCanvas, ctx, scenario, timeStep, meterToPixelFactor, mapLineColor, mapLineWidth);

    if( bTraces === true) 
    {
        var startStep = 0;
        if(timeStep > egoTraceLength) 
        {
            startStep = timeStep - egoTraceLength;
        }
        for(var j = startStep; j < timeStep; j+=2)
        {
            var pastEgoX = scenario["position"][j][0];
            var pastEgoY = scenario["position"][j][1];
            var pastEgoYaw = scenario["orientation"][j];
            var egoLength = scenario["dimension"][0];
            var egoWidth = scenario["dimension"][1];
            draw_global_pose_in_vehicle_frame(htmlCanvas, ctx, scenario, timeStep, meterToPixelFactor, pastEgoColor, 1.0,
                pastEgoX, pastEgoY, pastEgoYaw, egoLength, egoWidth);
        }
    }
    
    for(var i = 0; i < scenario["obstacles"][timeStep].length; i++)
    {
        var obstacleX = scenario["obstacles"][timeStep][i]["position"][0];
        var obstacleY = scenario["obstacles"][timeStep][i]["position"][1];
        var obstacleYaw = yaw_from_quaternion(scenario["obstacles"][timeStep][i]["orientation"]);
        var obstacleLength = scenario["obstacles"][timeStep][i]["dimension"][0];
        var obstacleWidth = scenario["obstacles"][timeStep][i]["dimension"][1];

        draw_global_pose_in_vehicle_frame(htmlCanvas, ctx, scenario, timeStep, meterToPixelFactor, obstacleColor, obstacleLineWidth,
            obstacleX, obstacleY, obstacleYaw, obstacleLength, obstacleWidth);
    }

    draw_ego_pose(htmlCanvas, ctx, scenario, timeStep, meterToPixelFactor, egoColor, egoLineWidth);

}

function yaw_from_quaternion(quanternionArray)
{
    t3 = +2.0 * (quanternionArray[3] * quanternionArray[2] + quanternionArray[0] * quanternionArray[1]);
    t4 = +1.0 - 2.0 * (quanternionArray[1] * quanternionArray[1] + quanternionArray[2] * quanternionArray[2]);
    yaw_z = Math.atan2(t3, t4);
     
    return yaw_z;
}

function convert_global_to_vehicle_frame(x, y, egoX, egoY, egoYaw)
{
    var dgx = x - egoX;
    var dgy = y - egoY;
    var dX = Math.sin(egoYaw - Math.atan2(dgy , dgx)) * Math.sqrt(Math.pow(dgx, 2) + Math.pow(dgy, 2));
    var dY = Math.cos(egoYaw - Math.atan2(dgy , dgx)) * Math.sqrt(Math.pow(dgx, 2) + Math.pow(dgy, 2));
     
    return [dX, dY];
}

function draw_ego_pose(canvas, ctx, scenario, timeStep, mTPF, color, lineWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;

    var egoYaw = scenario["orientation"][timeStep];
    var egoLength = scenario["dimension"][0];
    var egoWidth = scenario["dimension"][1];

    // delta x and y in length axis
    var d_length = [ Math.cos(egoYaw)*egoLength/2, Math.sin(egoYaw)*egoLength/2 ];
    // delta x and y in with axis
    var d_width = [ Math.cos(egoYaw - Math.PI/2)*egoWidth/2, Math.sin(egoYaw - Math.PI/2)*egoWidth/2 ];

    var ego0 = [+ d_length[0] + d_width[0], + d_length[1] + d_width[1]];
    var ego1 = [+ d_length[0] - d_width[0], + d_length[1] - d_width[1]];
    var ego2 = [- d_length[0] + d_width[0], - d_length[1] + d_width[1]];
    var ego3 = [- d_length[0] - d_width[0], - d_length[1] - d_width[1]];

    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * ego0[0], centerY + mTPF * ego0[1]);
    ctx.lineTo(centerX + mTPF * ego1[0], centerY + mTPF * ego1[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * ego0[0], centerY + mTPF * ego0[1]);
    ctx.lineTo(centerX + mTPF * ego2[0], centerY + mTPF * ego2[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * ego2[0], centerY + mTPF * ego2[1]);
    ctx.lineTo(centerX + mTPF * ego3[0], centerY + mTPF * ego3[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * ego3[0], centerY + mTPF * ego3[1]);
    ctx.lineTo(centerX + mTPF * ego1[0], centerY + mTPF * ego1[1]);
    ctx.stroke();
}

function draw_global_pose_in_vehicle_frame(canvas, ctx, scenario, timeStep, mTPF, color, lineWidth, poseX, poseY, poseYaw, poseLength, poseWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;

    var egoX = scenario["position"][timeStep][0];
    var egoY = scenario["position"][timeStep][1];
    var egoYaw = -Math.PI/2;

    // delta x and y in length axis
    var d_length = [ Math.cos(poseYaw)*poseLength/2, Math.sin(poseYaw)*poseLength/2 ];
    // delta x and y in with axis
    var d_width = [ Math.cos(poseYaw - Math.PI/2)*poseWidth/2, Math.sin(poseYaw - Math.PI/2)*poseWidth/2 ];

    var pos0 = convert_global_to_vehicle_frame(poseX + d_length[0] + d_width[0], poseY + d_length[1] + d_width[1], egoX, egoY, egoYaw);
    var pos1 = convert_global_to_vehicle_frame(poseX + d_length[0] - d_width[0], poseY + d_length[1] - d_width[1], egoX, egoY, egoYaw);
    var pos2 = convert_global_to_vehicle_frame(poseX - d_length[0] + d_width[0], poseY - d_length[1] + d_width[1], egoX, egoY, egoYaw);
    var pos3 = convert_global_to_vehicle_frame(poseX - d_length[0] - d_width[0], poseY - d_length[1] - d_width[1], egoX, egoY, egoYaw);

    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * pos0[0], centerY + mTPF * pos0[1]);
    ctx.lineTo(centerX + mTPF * pos1[0], centerY + mTPF * pos1[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * pos0[0], centerY + mTPF * pos0[1]);
    ctx.lineTo(centerX + mTPF * pos2[0], centerY + mTPF * pos2[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * pos2[0], centerY + mTPF * pos2[1]);
    ctx.lineTo(centerX + mTPF * pos3[0], centerY + mTPF * pos3[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX + mTPF * pos3[0], centerY + mTPF * pos3[1]);
    ctx.lineTo(centerX + mTPF * pos1[0], centerY + mTPF * pos1[1]);
    ctx.stroke();
}

function draw_map(canvas, ctx, scenario, timeStep, mTPF, lineColor, lineWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;

    var egoX = scenario["position"][timeStep][0];
    var egoY = scenario["position"][timeStep][1];
    var egoYaw = -Math.PI/2;

    
    ctx.strokeStyle = lineColor;
    ctx.lineWidth = lineWidth;

    let markers_index;
    for (markers_index in scenario_map["markers"]) {
        ctx.beginPath();
        let points_index;
        for (points_index in scenario_map["markers"][markers_index]["points"]) {
            var point = scenario_map["markers"][markers_index]["points"][points_index];
            var p = convert_global_to_vehicle_frame(point[0], point[1], egoX, egoY, egoYaw);
            if( points_index===0 ){
                ctx.moveTo(centerX + mTPF * p[0], centerY + mTPF * p[1]);
            }
            else
            {
                ctx.lineTo(centerX + mTPF * p[0], centerY + mTPF * p[1]);
            }
        }
        ctx.stroke();
    }
}
