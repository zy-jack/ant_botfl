

clc;
clear;

speedList = [3];

baseTrajectoryDir = fullfile(pwd, 'mobility_profile');
baseOutputDir = fullfile(pwd, 'perue_fading_bank_batch');

commonCfg.enbPos = [0.0, 0.0, 25.0];

commonCfg.fcHz = 3500e6;
commonCfg.traceDurationSec = 1200.0;
commonCfg.sampleIntervalSec = 2e-3;
commonCfg.numRBs = 100;

commonCfg.pathDelaysSec = [0, 80, 200, 500] * 1e-9;
commonCfg.avgPathGainsDb = [0, -12, -18, -24];
commonCfg.normalizePathGains = true;

commonCfg.losClusterTapIds = 1;
commonCfg.losTapPhaseOffsetsRad = 0.0;

commonCfg.randomSeed = 20260410;
commonCfg.internalSampleRateHz = [];

commonCfg.groups = {
    struct('name', 'rayleigh', 'isRayleigh', true, 'kDb', -Inf)
};

allOut = struct();

for i = 1:numel(speedList)
    speed = speedList(i);

    cfg = commonCfg;
    cfg.trajectoryCsv = fullfile(baseTrajectoryDir, sprintf('ue_trajectory_%d.csv', speed));
    cfg.outputDir = fullfile(baseOutputDir, sprintf('perue_fading_bank_v%d', speed));

    fprintf('\n====================================================\n');
    fprintf('Processing speed %d m/s\n', speed);
    fprintf('Trajectory file: %s\n', cfg.trajectoryCsv);
    fprintf('Output directory: %s\n', cfg.outputDir);
    fprintf('====================================================\n');

    out = generate_ns3_perue_mobility_aware_fading(cfg);

    allOut(i).speed = speed;
    allOut(i).out = out;
end

disp('Batch generation completed.');
disp(allOut);
