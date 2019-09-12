
"use strict";

let IONodeConfiguration = require('./IONodeConfiguration.js');
let JointLimits = require('./JointLimits.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let HeadState = require('./HeadState.js');
let IODeviceStatus = require('./IODeviceStatus.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let AnalogIOState = require('./AnalogIOState.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let EndpointStates = require('./EndpointStates.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let IONodeStatus = require('./IONodeStatus.js');
let InteractionControlState = require('./InteractionControlState.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let NavigatorState = require('./NavigatorState.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let SEAJointState = require('./SEAJointState.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let HomingState = require('./HomingState.js');
let DigitalIOState = require('./DigitalIOState.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let EndpointState = require('./EndpointState.js');
let NavigatorStates = require('./NavigatorStates.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let IODataStatus = require('./IODataStatus.js');
let CameraSettings = require('./CameraSettings.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let IOStatus = require('./IOStatus.js');
let HomingCommand = require('./HomingCommand.js');
let CameraControl = require('./CameraControl.js');
let JointCommand = require('./JointCommand.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');

module.exports = {
  IONodeConfiguration: IONodeConfiguration,
  JointLimits: JointLimits,
  CollisionAvoidanceState: CollisionAvoidanceState,
  HeadState: HeadState,
  IODeviceStatus: IODeviceStatus,
  CollisionDetectionState: CollisionDetectionState,
  EndpointNamesArray: EndpointNamesArray,
  RobotAssemblyState: RobotAssemblyState,
  AnalogOutputCommand: AnalogOutputCommand,
  AnalogIOState: AnalogIOState,
  IOComponentStatus: IOComponentStatus,
  EndpointStates: EndpointStates,
  IOComponentCommand: IOComponentCommand,
  IONodeStatus: IONodeStatus,
  InteractionControlState: InteractionControlState,
  InteractionControlCommand: InteractionControlCommand,
  IODeviceConfiguration: IODeviceConfiguration,
  NavigatorState: NavigatorState,
  HeadPanCommand: HeadPanCommand,
  SEAJointState: SEAJointState,
  URDFConfiguration: URDFConfiguration,
  HomingState: HomingState,
  DigitalIOState: DigitalIOState,
  AnalogIOStates: AnalogIOStates,
  EndpointState: EndpointState,
  NavigatorStates: NavigatorStates,
  IOComponentConfiguration: IOComponentConfiguration,
  IODataStatus: IODataStatus,
  CameraSettings: CameraSettings,
  DigitalOutputCommand: DigitalOutputCommand,
  IOStatus: IOStatus,
  HomingCommand: HomingCommand,
  CameraControl: CameraControl,
  JointCommand: JointCommand,
  DigitalIOStates: DigitalIOStates,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandAction: CalibrationCommandAction,
  CalibrationCommandResult: CalibrationCommandResult,
};
