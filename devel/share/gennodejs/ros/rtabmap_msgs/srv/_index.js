
"use strict";

let GetPlan = require('./GetPlan.js')
let GlobalBundleAdjustment = require('./GlobalBundleAdjustment.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let ResetPose = require('./ResetPose.js')
let GetMap2 = require('./GetMap2.js')
let SetLabel = require('./SetLabel.js')
let PublishMap = require('./PublishMap.js')
let AddLink = require('./AddLink.js')
let RemoveLabel = require('./RemoveLabel.js')
let GetNodeData = require('./GetNodeData.js')
let DetectMoreLoopClosures = require('./DetectMoreLoopClosures.js')
let LoadDatabase = require('./LoadDatabase.js')
let CleanupLocalGrids = require('./CleanupLocalGrids.js')
let ListLabels = require('./ListLabels.js')
let SetGoal = require('./SetGoal.js')
let GetMap = require('./GetMap.js')

module.exports = {
  GetPlan: GetPlan,
  GlobalBundleAdjustment: GlobalBundleAdjustment,
  GetNodesInRadius: GetNodesInRadius,
  ResetPose: ResetPose,
  GetMap2: GetMap2,
  SetLabel: SetLabel,
  PublishMap: PublishMap,
  AddLink: AddLink,
  RemoveLabel: RemoveLabel,
  GetNodeData: GetNodeData,
  DetectMoreLoopClosures: DetectMoreLoopClosures,
  LoadDatabase: LoadDatabase,
  CleanupLocalGrids: CleanupLocalGrids,
  ListLabels: ListLabels,
  SetGoal: SetGoal,
  GetMap: GetMap,
};
