const fileSelector = document.getElementById('file-selector');
  fileSelector.addEventListener('change', (event) => {
    const fileList = event.target.files;
    console.log(fileList);
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
        // drawCharts(scenario_content);
        
        for(var i = 0; i < scenario_content["timeSteps"].length; i++) 
        {
            drawVideo(scenario_content, i);
        }
    });
    console.log(file);
    reader.readAsText(file);
}

function drawCharts(scenario)
{
    clear_Canavs();
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
}

function drawVideo(scenario, timeStep)
{
    var htmlCanvas = document.getElementById("scenarioCanvs");
    var ctx = htmlCanvas.getContext("2d");

    // clear the canvas
    ctx.beginPath();
    ctx.clearRect(0, 0, htmlCanvas.width, htmlCanvas.height);

    console.log(">> new frame");
    var egoX = scenario["position"][timeStep][0];
    var egoY = scenario["position"][timeStep][1];
    var egoYaw = scenario["orientation"][timeStep];
    console.log(egoX, egoY, egoYaw);

    for(var i = 0; i < scenario["obstacles"][timeStep].length; i++)
    {
        console.log(scenario["obstacles"][timeStep][i]["position"]);
        console.log(yaw_from_quaternion(scenario["obstacles"][timeStep][i]["orientation"]));
        // ctx.beginPath();
        // ctx.moveTo(this.from.x, this.from.y);
        // ctx.lineTo(this.to.x, this.to.y);
        // ctx.stroke();
    }
}

function yaw_from_quaternion(quanternionArray)
{
    t3 = +2.0 * (quanternionArray[3] * quanternionArray[2] + quanternionArray[0] * quanternionArray[1]);
    t4 = +1.0 - 2.0 * (quanternionArray[1] * quanternionArray[1] + quanternionArray[2] * quanternionArray[2]);
    yaw_z = Math.atan2(t3, t4);
     
    return yaw_z;
}
        
