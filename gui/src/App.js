import React from "react";
import "./App.css";

const App = () => {
  return (
    <div className="app-container">
      <div className="logo-section">
        <img src={require('./assets/images/logo.png')} alt="Logo" className="logo" />
        <h1 className="title">SPEAR DASHBOARD</h1>
      </div>


      <div className="section initial">
        <div className="button">Initialize Arm</div>
        <div className="button">Initialize Rover</div>
        <div className="button red">Autonomy Drive Mode</div>
      </div>

      <div className="section">
        <div className="button">Manual GPS Locator</div>
        <div className="button">ArUCO GPS Locator</div>
        <div className="button">Image Capture</div>
        <div className="button">Panorama Stitcher</div>
      </div>

      <div className="section">
        <div className="button">Camera 1</div>
        <div className="button">Camera 2</div>
        <div className="button">Camera 3</div>
        <div className="button">Camera 4</div>
      </div>

      <div className="section">
        <div className="button">Science Team 1</div>
        <div className="button">Science Team 2</div>
        <div className="button">GPS Data Recorder</div>
      </div>

      <div className="bottom-section">
        <input type="text" placeholder="Enter GPS Information" className="text-field" />
        <div className="button update">Update</div>
      </div>
    </div>
  );
};

export default App;
