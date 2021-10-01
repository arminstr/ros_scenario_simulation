// ros_scenario_simulation
// Visualization of CI Framework
// Copyright 2021, Armin Straller,
// University of Applied Sciences Augsburg
// All rights reserved.
// Licensed under the MIT license

import {menuApi} from "./menu.js";
import {scenarioApi} from "./scenario.js";

const menu = menuApi.createMenu();
const scenario = scenarioApi.createScenarioDisplay();
// const fileSelector = document.getElementById('file-selector');
//   fileSelector.addEventListener('change', (event) => {
//     const fileList = event.target.files;
//     console.log(fileList);
//     clearInterval(timerId);
//     readScenarioJson(fileList[0]);
// });
fetch("scenarios.json")
    .then(response => response.json())
    .then(async data => {
        for (let path_i in data.scenario_json_paths)
        {
            const item = menuApi.createItem (data.scenario_json_paths[path_i], async (m) => {
                loadScenarioJson(data.scenario_json_paths[path_i]);
            });
            let fetched_data = await (await fetch(data.scenario_json_paths[path_i])).json();
            if( fetched_data.collision > 0 || fetched_data.stopTrigger) 
            {
                item.indicatorDiv.style.backgroundColor = "red";
            } else {
                item.indicatorDiv.style.backgroundColor = "green";
            }
            menu.addItem(item);
        }
        menu.show("menuDiv");
    });


function loadScenarioJson(path)
{
    fetch(path)
    .then(response => response.json())
    .then(data => {
        scenario.show("scenarioDiv");
        scenario.setScenarioData(data);
        scenario.fillCostTable();
        scenario.startVideo();
        scenario.drawCharts();
    });
}

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
        
    });
    console.log(file);
    reader.readAsText(file);
}

function DataWriter(scenario_content){
    drawCharts(scenario_content);
    startVideo(scenario_content);
    fillCostTable(scenario_content);
    return this;
}
