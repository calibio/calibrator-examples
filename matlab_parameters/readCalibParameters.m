%readCalibParameters(fileName) reads a json parameter file generated with
% Calib Camera Calibrator (v. 1.5). 
%
% The resulting structs can either be used directly or
% cameraIntrinsics()/cameraParameters()/stereoParameters()
% objects can be created using them if CV Toolbox is available.
% 
% (c) Calib.io ApS
%
% Note: coordinates are adjusted to Matlab's 1-based index convention
   
function [intrinsics, extrinsics] = readCalibParameters(fileName)

jsonStruct = jsondecode(fileread(fileName));

nCameras = length(jsonStruct.Calibration.cameras);

assert(nCameras > 0)
assert(strcmp(jsonStruct.Calibration.cameras(1).model.polymorphic_name, 'libCalib::CameraModelMatlab'));

for i=1:nCameras
    
   h = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.CameraModelCRT.CameraModelBase.imageHeight;
   w = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.CameraModelCRT.CameraModelBase.imageWidth;

   f = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.f.val;
   ar = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.ar.val;
   cx = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.cx.val;
   cy = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.cy.val;
   k1 = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.k1.val;
   k2 = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.k2.val;
   k3 = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.k3.val;
   p1 = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.p1.val;
   p2 = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.p2.val;
   skew = jsonStruct.Calibration.cameras(i).model.ptr_wrapper.data.parameters.skew.val;

   intrinsics(i).RadialDistortion = [k1 k2 k3];
   intrinsics(i).TangentialDistortion =  [p1 p2];
   intrinsics(i).ImageSize = [w h];
   intrinsics(i).IntrinsicMatrix = [f, 0, 0; skew, f*ar, 0; cx+1, cy+1, 1.0]; 
   
   extrinsics(i).RotationVector = cell2mat(struct2cell(jsonStruct.Calibration.cameras(i).transform.rotation));
   extrinsics(i).TranslationVector = cell2mat(struct2cell(jsonStruct.Calibration.cameras(i).transform.translation));
   
end

end

