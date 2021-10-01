/**
 * Classes used for Scenario Printing to Screen
 * @author Armin Straller <armin.straller@hs-augsburg.de>
 */

export let scenarioApi = new ScenarioApi();

function ScenarioApi(){}

ScenarioApi .prototype .createScenarioDisplay = function () {
    return new ScenarioDisplay();
}

function ScenarioDisplay(){
    this.timerId = 0;
    this.bHidden = true;
    this.scenarioDiv = null;
    this.scenario = null;
    return this;
}

ScenarioDisplay .prototype .show = function (id) {
    this.scenarioDiv = document.getElementById(id);
    this.closeButton = document.getElementById("closeScenario");
    this.closeButton.addEventListener("click", (event)=>{
        this.hide();
    });

    // getting all DOM elements from report.html
    this.htmlCanvas = document.getElementById("scenarioCanvas");
    this.meterToPixelFactor = this.htmlCanvas.height / 40.0;
    this.zoomInButton = document.getElementById("scenarioCanvasZoomIn");
    this.zoomInButton.addEventListener("click", (event) => {
        this.meterToPixelFactor *= 1.1;
    });
    this.zoomOutButton = document.getElementById("scenarioCanvasZoomOut");
    this.zoomOutButton.addEventListener("click", (event) => {
        this.meterToPixelFactor *= 0.9;
    });

    this.timeCostTD = document.getElementById("timeCost");
    this.collisionCostTD = document.getElementById("collisionCost");
    this.stopTriggerCostTD = document.getElementById("stopTriggerCost");
    this.pathLengthCostTD = document.getElementById("pathLengthCost");
    this.accelerationCostTD = document.getElementById("accelerationCost");
    this.jerkCostTD = document.getElementById("jerkCost");
    this.steeringAngleCostTD = document.getElementById("steeringAngleCost");
    this.steeringRateCostTD = document.getElementById("steeringRateCost");
    this.yawRateCostTD = document.getElementById("yawRateCost");
    this.dTObstaclesCostTD = document.getElementById("dTObstaclesCost");
    this.dTCenterLineCostTD = document.getElementById("dTCenterLineCost");
    this.costSumTD = document.getElementById("costSum");

    this.timeWeightedCostTD = document.getElementById("timeWeightedCost");
    this.collisionWeightedCostTD = document.getElementById("collisionWeightedCost");
    this.stopTriggerWeightedCostTD = document.getElementById("stopTriggerWeightedCost");
    this.pathLengthWeightedCostTD = document.getElementById("pathLengthWeightedCost");
    this.accelerationWeightedCostTD = document.getElementById("accelerationWeightedCost");
    this.jerkWeightedCostTD = document.getElementById("jerkWeightedCost");
    this.steeringAngleWeightedCostTD = document.getElementById("steeringAngleWeightedCost");
    this.steeringRateWeightedCostTD = document.getElementById("steeringRateWeightedCost");
    this.yawRateWeightedCostTD = document.getElementById("yawRateWeightedCost");
    this.dTObstaclesWeightedCostTD = document.getElementById("dTObstaclesWeightedCost");
    this.dTCenterLineWeightedCostTD = document.getElementById("dTCenterLineWeightedCost");
    this.costSumWeightedTD = document.getElementById("costSumWeighted");

    this.vel_div = document.getElementById('velocityChart');
    this.accel_div = document.getElementById('accelerationChart');
    this.jerk_div = document.getElementById('jerkChart');
    this.steer_div = document.getElementById('steeringChart');
    this.steering_rate_div = document.getElementById('steeringRateChart');
    this.yaw_rate_div = document.getElementById('yawRateChart');

    this.bHidden = false; 
    this.scenarioDiv.hidden = false;
}

ScenarioDisplay .prototype .hide = function () {
    this.scenarioDiv.hidden = true;
    this.bHidden = true;
}

ScenarioDisplay .prototype .setScenarioData = function (data) {
    clearInterval(this.timerId);
    this.scenario = data;
}

ScenarioDisplay .prototype .fillCostTable = function () 
{
    this.timeCostTD.innerHTML = (Math.round(this.scenario["time"] * 100) / 100).toFixed(2);
    this.collisionCostTD.innerHTML = (Math.round(this.scenario["collision"] * 100) / 100).toFixed(2);
    this.stopTriggerCostTD.innerHTML = (Math.round(this.scenario["stopTrigger"] * 100) / 100).toFixed(2);
    this.pathLengthCostTD.innerHTML = (Math.round(this.scenario["pathLength"] * 100) / 100).toFixed(2);
    this.accelerationCostTD.innerHTML = (Math.round(this.scenario["accelerationCost"] * 100) / 100).toFixed(2);
    this.jerkCostTD.innerHTML = (Math.round(this.scenario["jerkCost"] * 100) / 100).toFixed(2);
    this.steeringAngleCostTD.innerHTML = (Math.round(this.scenario["steeringAngleCost"] * 100) / 100).toFixed(2);
    this.steeringRateCostTD.innerHTML = (Math.round(this.scenario["steeringRateCost"] * 100) / 100).toFixed(2);
    this.yawRateCostTD.innerHTML = (Math.round(this.scenario["yawRateCost"] * 100) / 100).toFixed(2);
    this.dTObstaclesCostTD.innerHTML = (Math.round(this.scenario["distanceToObstaclesCost"] * 100) / 100).toFixed(2);
    this.dTCenterLineCostTD.innerHTML = (Math.round(this.scenario["distanceToCenterLinesCost"] * 100) / 100).toFixed(2);
    this.costSumTD.innerHTML = (Math.round(this.scenario["costSum"] * 100) / 100).toFixed(2);

    this.timeWeightedCostTD.innerHTML = (Math.round(this.scenario["timeWeighted"] * 100) / 100).toFixed(2);
    this.collisionWeightedCostTD.innerHTML = (Math.round(this.scenario["collisionWeighted"] * 100) / 100).toFixed(2);
    this.stopTriggerWeightedCostTD.innerHTML = (Math.round(this.scenario["stopTriggerWeighted"] * 100) / 100).toFixed(2);
    this.pathLengthWeightedCostTD.innerHTML = (Math.round(this.scenario["pathLengthWeighted"] * 100) / 100).toFixed(2);
    this.accelerationWeightedCostTD.innerHTML = (Math.round(this.scenario["accelerationCostWeighted"] * 100) / 100).toFixed(2);
    this.jerkWeightedCostTD.innerHTML = (Math.round(this.scenario["jerkCostWeighted"] * 100) / 100).toFixed(2);
    this.steeringAngleWeightedCostTD.innerHTML = (Math.round(this.scenario["steeringAngleCostWeighted"] * 100) / 100).toFixed(2);
    this.steeringRateWeightedCostTD.innerHTML = (Math.round(this.scenario["steeringRateCostWeighted"] * 100) / 100).toFixed(2);
    this.yawRateWeightedCostTD.innerHTML = (Math.round(this.scenario["yawRateCostWeighted"] * 100) / 100).toFixed(2);
    this.dTObstaclesWeightedCostTD.innerHTML = (Math.round(this.scenario["distanceToObstaclesCostWeighted"] * 100) / 100).toFixed(2);
    this.dTCenterLineWeightedCostTD.innerHTML = (Math.round(this.scenario["distanceToCenterLinesCostWeighted"] * 100) / 100).toFixed(2);
    this.costSumWeightedTD.innerHTML = (Math.round(this.scenario["costSumWeighted"] * 100) / 100).toFixed(2);
}


ScenarioDisplay .prototype .drawVideo = function (timeStep)
{
    var ctx = this.htmlCanvas.getContext("2d");
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
    ctx.clearRect(0, 0, this.htmlCanvas.width, this.htmlCanvas.height);

    this.draw_map(this.htmlCanvas, ctx, timeStep, mapLineColor, mapLineWidth);

    if( bTraces === true) 
    {
        var startStep = 0;
        if(timeStep > egoTraceLength) 
        {
            startStep = timeStep - egoTraceLength;
        }
        for(var j = startStep; j < timeStep; j+=2)
        {
            var pastEgoX = this.scenario["position"][j][0];
            var pastEgoY = this.scenario["position"][j][1];
            var pastEgoYaw = this.scenario["orientation"][j];
            var egoLength = this.scenario["dimension"][0];
            var egoWidth = this.scenario["dimension"][1];
            this.draw_global_pose_in_vehicle_frame(this.htmlCanvas, ctx, timeStep, pastEgoColor, 1.0,
                pastEgoX, pastEgoY, pastEgoYaw, egoLength, egoWidth);
        }
    }
    
    for(var i = 0; i < this.scenario["obstacles"][timeStep].length; i++)
    {
        var obstacleX = this.scenario["obstacles"][timeStep][i]["position"][0];
        var obstacleY = this.scenario["obstacles"][timeStep][i]["position"][1];
        var obstacleYaw = this.scenario["obstacles"][timeStep][i]["orientation"];
        var obstacleLength = this.scenario["obstacles"][timeStep][i]["dimension"][0];
        var obstacleWidth = this.scenario["obstacles"][timeStep][i]["dimension"][1];

        this.draw_global_pose_in_vehicle_frame(this.htmlCanvas, ctx, timeStep, obstacleColor, obstacleLineWidth,
            obstacleX, obstacleY, obstacleYaw, obstacleLength, obstacleWidth);
    }

    this.draw_ego_pose(this.htmlCanvas, ctx, timeStep, egoColor, egoLineWidth);

}

ScenarioDisplay .prototype .startVideo = function (){
    let timeStep = 0;
    this.timerId = setInterval(()=>{
        this.drawVideo(timeStep);
        timeStep++;
        if(timeStep >= this.scenario["timeSteps"].length)
        {
            clearInterval(this.timerId);
            this.startVideo();
        }
    }, 100);
}

ScenarioDisplay .prototype .drawCharts = function ()
{
    // Define Data
    var vel_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["velocity"],
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
    Plotly.newPlot(this.vel_div, vel_data, vel_layout);

    
    // Define Data
    var accel_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["acceleration"],
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
    Plotly.newPlot(this.accel_div, accel_data, accel_layout);

    
    // Define Data
    var jerk_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["jerk"],
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
    Plotly.newPlot(this.jerk_div, jerk_data, jerk_layout);

    // Define Data
    var steer_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["steeringAngle"],
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
    Plotly.newPlot(this.steer_div, steer_data, steer_layout);

    
    // Define Data
    var steering_rate_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["steeringRate"],
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
    Plotly.newPlot(this.steering_rate_div, steering_rate_data, steering_rate_layout);

    
    // Define Data
    var yaw_rate_data = [{
        x: this.scenario["timeSteps"],
        y: this.scenario["yawRate"],
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
    Plotly.newPlot(this.yaw_rate_div, yaw_rate_data, yaw_rate_layout);
}

ScenarioDisplay .prototype .convert_global_to_vehicle_frame = function (x, y, egoX, egoY, egoYaw)
{
    var dgx = x - egoX;
    var dgy = y - egoY;
    var dX = Math.sin(egoYaw - Math.atan2(dgy , dgx)) * Math.sqrt(Math.pow(dgx, 2) + Math.pow(dgy, 2));
    var dY = Math.cos(egoYaw - Math.atan2(dgy , dgx)) * Math.sqrt(Math.pow(dgx, 2) + Math.pow(dgy, 2));
     
    return [dX, dY];
}

ScenarioDisplay .prototype .draw_ego_pose = function (canvas, ctx, timeStep, color, lineWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;

    var egoYaw = this.scenario["orientation"][timeStep];
    var egoLength = this.scenario["dimension"][0];
    var egoWidth = this.scenario["dimension"][1];

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
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * ego0[0]), centerY + this.meterToPixelFactor * ego0[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * ego1[0]), centerY + this.meterToPixelFactor * ego1[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * ego0[0]), centerY + this.meterToPixelFactor * ego0[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * ego2[0]), centerY + this.meterToPixelFactor * ego2[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * ego2[0]), centerY + this.meterToPixelFactor * ego2[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * ego3[0]), centerY + this.meterToPixelFactor * ego3[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * ego3[0]), centerY + this.meterToPixelFactor * ego3[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * ego1[0]), centerY + this.meterToPixelFactor * ego1[1]);
    ctx.stroke();
}

ScenarioDisplay .prototype .draw_global_pose_in_vehicle_frame = function(canvas, ctx, timeStep, color, lineWidth, poseX, poseY, poseYaw, poseLength, poseWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;
    var egoX = this.scenario["position"][timeStep][0];
    var egoY = this.scenario["position"][timeStep][1];
    var egoYaw = -Math.PI/2;

    // delta x and y in length axis
    var d_length = [ Math.cos(poseYaw)*poseLength/2, Math.sin(poseYaw)*poseLength/2 ];
    // delta x and y in with axis
    var d_width = [ Math.cos(poseYaw - Math.PI/2)*poseWidth/2, Math.sin(poseYaw - Math.PI/2)*poseWidth/2 ];

    var pos0 = this.convert_global_to_vehicle_frame(poseX + d_length[0] + d_width[0], poseY + d_length[1] + d_width[1], egoX, egoY, egoYaw);
    var pos1 = this.convert_global_to_vehicle_frame(poseX + d_length[0] - d_width[0], poseY + d_length[1] - d_width[1], egoX, egoY, egoYaw);
    var pos2 = this.convert_global_to_vehicle_frame(poseX - d_length[0] + d_width[0], poseY - d_length[1] + d_width[1], egoX, egoY, egoYaw);
    var pos3 = this.convert_global_to_vehicle_frame(poseX - d_length[0] - d_width[0], poseY - d_length[1] - d_width[1], egoX, egoY, egoYaw);

    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * pos0[0]), centerY + this.meterToPixelFactor * pos0[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * pos1[0]), centerY + this.meterToPixelFactor * pos1[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * pos0[0]), centerY + this.meterToPixelFactor * pos0[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * pos2[0]), centerY + this.meterToPixelFactor * pos2[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * pos2[0]), centerY + this.meterToPixelFactor * pos2[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * pos3[0]), centerY + this.meterToPixelFactor * pos3[1]);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * pos3[0]), centerY + this.meterToPixelFactor * pos3[1]);
    ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * pos1[0]), centerY + this.meterToPixelFactor * pos1[1]);
    ctx.stroke();
}

ScenarioDisplay .prototype .draw_map = function(canvas, ctx, timeStep, lineColor, lineWidth) 
{
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;

    var egoX = this.scenario["position"][timeStep][0];
    var egoY = this.scenario["position"][timeStep][1];
    var egoYaw = -Math.PI/2;

    
    ctx.strokeStyle = lineColor;
    ctx.lineWidth = lineWidth;

    let markers_index;
    for (markers_index in scenario_map["markers"]) {
        ctx.beginPath();
        let points_index;
        for (points_index in scenario_map["markers"][markers_index]["points"]) {
            var point = scenario_map["markers"][markers_index]["points"][points_index];
            var p = this.convert_global_to_vehicle_frame(point[0], point[1], egoX, egoY, egoYaw);
            if( points_index===0 ){
                ctx.moveTo(canvas.width - (centerX + this.meterToPixelFactor * p[0]), centerY + this.meterToPixelFactor * p[1]);
            }
            else
            {
                ctx.lineTo(canvas.width - (centerX + this.meterToPixelFactor * p[0]), centerY + this.meterToPixelFactor * p[1]);
            }
        }
        ctx.stroke();
    }
}
