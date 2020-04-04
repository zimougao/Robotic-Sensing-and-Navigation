% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 3093.260070296225422 ; 3109.216728075006358 ];

%-- Principal point:
cc = [ 2016.459161300047754 ; 1498.991902602678010 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.114098285871720 ; -0.309914536543184 ; -0.000409562538880 ; -0.000757478049100 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 10.894589037597834 ; 11.358374287951728 ];

%-- Principal point uncertainty:
cc_error = [ 7.413180623860907 ; 6.316245341910881 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008045666353289 ; 0.024725350167152 ; 0.000722601740753 ; 0.000905175680580 ; 0.000000000000000 ];

%-- Image size:
nx = 4032;
ny = 3024;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 20;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.687488e+00 ; 2.388268e+00 ; 7.641210e-02 ];
Tc_1  = [ -3.875868e+01 ; -7.878812e+01 ; 2.165548e+02 ];
omc_error_1 = [ 2.259787e-03 ; 3.027072e-03 ; 5.013326e-03 ];
Tc_error_1  = [ 5.300147e-01 ; 4.430664e-01 ; 8.253634e-01 ];

%-- Image #2:
omc_2 = [ 1.740881e+00 ; 2.399077e+00 ; -3.374316e-02 ];
Tc_2  = [ -3.936267e+01 ; -7.878717e+01 ; 2.213482e+02 ];
omc_error_2 = [ 2.327611e-03 ; 3.038688e-03 ; 5.208631e-03 ];
Tc_error_2  = [ 5.387760e-01 ; 4.495246e-01 ; 8.398454e-01 ];

%-- Image #3:
omc_3 = [ 1.722248e+00 ; 2.428532e+00 ; -1.076038e-01 ];
Tc_3  = [ -3.900835e+01 ; -8.027174e+01 ; 2.323125e+02 ];
omc_error_3 = [ 2.388158e-03 ; 3.190669e-03 ; 5.463267e-03 ];
Tc_error_3  = [ 5.633183e-01 ; 4.692178e-01 ; 8.755099e-01 ];

%-- Image #4:
omc_4 = [ 1.719994e+00 ; 2.538095e+00 ; -2.407364e-01 ];
Tc_4  = [ -2.549812e+01 ; -8.487389e+01 ; 2.472121e+02 ];
omc_error_4 = [ 2.609773e-03 ; 3.406750e-03 ; 6.082308e-03 ];
Tc_error_4  = [ 5.986097e-01 ; 4.953842e-01 ; 9.240727e-01 ];

%-- Image #5:
omc_5 = [ 1.737228e+00 ; 2.564925e+00 ; -2.524695e-01 ];
Tc_5  = [ -5.121826e+01 ; -9.070991e+01 ; 2.361004e+02 ];
omc_error_5 = [ 2.253588e-03 ; 3.279731e-03 ; 5.616639e-03 ];
Tc_error_5  = [ 5.727084e-01 ; 4.751670e-01 ; 8.627163e-01 ];

%-- Image #6:
omc_6 = [ -1.758878e+00 ; -2.515347e+00 ; 2.552263e-01 ];
Tc_6  = [ -4.874940e+01 ; -8.927371e+01 ; 2.361601e+02 ];
omc_error_6 = [ 2.393429e-03 ; 3.162367e-03 ; 5.647800e-03 ];
Tc_error_6  = [ 5.734148e-01 ; 4.762671e-01 ; 8.602446e-01 ];

%-- Image #7:
omc_7 = [ -1.839670e+00 ; -2.404609e+00 ; 1.295995e-01 ];
Tc_7  = [ -5.812239e+01 ; -7.558109e+01 ; 2.289385e+02 ];
omc_error_7 = [ 2.429898e-03 ; 3.256063e-03 ; 5.680078e-03 ];
Tc_error_7  = [ 5.551632e-01 ; 4.699138e-01 ; 8.544838e-01 ];

%-- Image #8:
omc_8 = [ -1.942840e+00 ; -2.270062e+00 ; -1.009174e-01 ];
Tc_8  = [ -6.612637e+01 ; -7.022591e+01 ; 2.079971e+02 ];
omc_error_8 = [ 2.398731e-03 ; 2.971162e-03 ; 5.415923e-03 ];
Tc_error_8  = [ 5.111173e-01 ; 4.394871e-01 ; 7.928734e-01 ];

%-- Image #9:
omc_9 = [ -1.989938e+00 ; -2.246637e+00 ; -2.017505e-01 ];
Tc_9  = [ -6.060192e+01 ; -5.716376e+01 ; 1.951409e+02 ];
omc_error_9 = [ 2.161147e-03 ; 2.830240e-03 ; 5.147516e-03 ];
Tc_error_9  = [ 4.797420e-01 ; 4.126261e-01 ; 7.521404e-01 ];

%-- Image #10:
omc_10 = [ -2.062265e+00 ; -2.314802e+00 ; -3.614172e-01 ];
Tc_10  = [ -5.336754e+01 ; -5.537976e+01 ; 1.744257e+02 ];
omc_error_10 = [ 1.960203e-03 ; 2.634298e-03 ; 5.004299e-03 ];
Tc_error_10  = [ 4.333728e-01 ; 3.699787e-01 ; 6.871989e-01 ];

%-- Image #11:
omc_11 = [ 1.871917e+00 ; 1.952612e+00 ; -4.021484e-01 ];
Tc_11  = [ -6.979852e+01 ; -6.908800e+01 ; 2.256280e+02 ];
omc_error_11 = [ 1.866909e-03 ; 2.395465e-03 ; 3.857366e-03 ];
Tc_error_11  = [ 5.439520e-01 ; 4.526579e-01 ; 7.336943e-01 ];

%-- Image #12:
omc_12 = [ 1.693044e+00 ; 2.188199e+00 ; -4.651488e-01 ];
Tc_12  = [ -5.618869e+01 ; -8.569109e+01 ; 2.235690e+02 ];
omc_error_12 = [ 1.635249e-03 ; 2.494221e-03 ; 3.877191e-03 ];
Tc_error_12  = [ 5.420440e-01 ; 4.464916e-01 ; 7.099376e-01 ];

%-- Image #13:
omc_13 = [ 1.685917e+00 ; 2.394249e+00 ; -4.320982e-01 ];
Tc_13  = [ -5.842328e+01 ; -9.320734e+01 ; 2.245037e+02 ];
omc_error_13 = [ 1.663959e-03 ; 2.735390e-03 ; 4.373592e-03 ];
Tc_error_13  = [ 5.470554e-01 ; 4.498572e-01 ; 7.502430e-01 ];

%-- Image #14:
omc_14 = [ -1.955383e+00 ; -2.137963e+00 ; 3.258316e-01 ];
Tc_14  = [ -8.099518e+01 ; -7.088573e+01 ; 2.313144e+02 ];
omc_error_14 = [ 2.359860e-03 ; 2.407348e-03 ; 4.598518e-03 ];
Tc_error_14  = [ 5.588477e-01 ; 4.715275e-01 ; 7.840131e-01 ];

%-- Image #15:
omc_15 = [ -2.072754e+00 ; -1.804124e+00 ; 9.995983e-02 ];
Tc_15  = [ -9.701372e+01 ; -3.821075e+01 ; 2.149067e+02 ];
omc_error_15 = [ 2.247397e-03 ; 2.223725e-03 ; 4.104216e-03 ];
Tc_error_15  = [ 5.163974e-01 ; 4.472614e-01 ; 7.333600e-01 ];

%-- Image #16:
omc_16 = [ -2.076074e+00 ; -1.737426e+00 ; -1.881330e-01 ];
Tc_16  = [ -1.049515e+02 ; -1.565189e+01 ; 1.944793e+02 ];
omc_error_16 = [ 2.120215e-03 ; 2.295943e-03 ; 4.036284e-03 ];
Tc_error_16  = [ 4.677067e-01 ; 4.129731e-01 ; 7.179466e-01 ];

%-- Image #17:
omc_17 = [ -2.056394e+00 ; -1.792766e+00 ; -3.942882e-01 ];
Tc_17  = [ -8.646985e+01 ; -1.988433e+01 ; 1.762745e+02 ];
omc_error_17 = [ 1.948412e-03 ; 2.339646e-03 ; 3.992197e-03 ];
Tc_error_17  = [ 4.271602e-01 ; 3.728956e-01 ; 6.701091e-01 ];

%-- Image #18:
omc_18 = [ -1.912261e+00 ; -1.918844e+00 ; -6.208811e-01 ];
Tc_18  = [ -6.585618e+01 ; -1.512275e+01 ; 1.456389e+02 ];
omc_error_18 = [ 1.666713e-03 ; 2.484888e-03 ; 3.860942e-03 ];
Tc_error_18  = [ 3.525973e-01 ; 3.078164e-01 ; 5.860718e-01 ];

%-- Image #19:
omc_19 = [ -1.785053e+00 ; -2.185861e+00 ; -6.348872e-01 ];
Tc_19  = [ -4.065427e+01 ; -2.928445e+01 ; 1.712131e+02 ];
omc_error_19 = [ 1.620251e-03 ; 2.722857e-03 ; 4.275008e-03 ];
Tc_error_19  = [ 4.160900e-01 ; 3.594889e-01 ; 6.986914e-01 ];

%-- Image #20:
omc_20 = [ -1.627088e+00 ; -2.383804e+00 ; -7.859479e-01 ];
Tc_20  = [ -1.840497e+01 ; -4.160373e+01 ; 1.522016e+02 ];
omc_error_20 = [ 1.454418e-03 ; 2.796199e-03 ; 4.189157e-03 ];
Tc_error_20  = [ 3.720594e-01 ; 3.216827e-01 ; 6.476053e-01 ];

