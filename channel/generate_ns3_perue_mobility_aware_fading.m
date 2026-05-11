function out = generate_ns3_perue_mobility_aware_fading(cfg)


    cfg = validateAndCompleteCfg(cfg);
    rng(cfg.randomSeed, 'twister');

    raw = readtable(cfg.trajectoryCsv);

    required = {'time', 'ueIndex', 'nodeId', 'x', 'y', 'z', 'speed', 'headingRad'};
    for i = 1:numel(required)
        if ~ismember(required{i}, raw.Properties.VariableNames)
            error('trajectory CSV is missing column: %s', required{i});
        end
    end

    nodeIds = unique(raw.nodeId);
    t = (0:cfg.sampleIntervalSec:cfg.traceDurationSec - cfg.sampleIntervalSec).';
    samplesNum = numel(t);


    rbBwHz = 180e3;
    fRbHz = ((0:cfg.numRBs - 1) - (cfg.numRBs - 1) / 2) * rbBwHz;
    A = exp(-1j * 2 * pi * (fRbHz(:) * cfg.pathDelaysSec(:).'));

    pathPowersLin = getPathPowerLinear(cfg);


    if ~exist(cfg.outputDir, 'dir')
        mkdir(cfg.outputDir);
    end
    for i = 1:numel(cfg.groups)
        groupDir = fullfile(cfg.outputDir, cfg.groups{i}.name);
        if ~exist(groupDir, 'dir')
            mkdir(groupDir);
        end
    end

    out.nodeIds = nodeIds;
    out.samplesNum = samplesNum;
    out.files = {};

    for iNode = 1:numel(nodeIds)
        nodeId = nodeIds(iNode);

        motion = buildMotionProfileForNode(cfg, raw, nodeId, t);


        fdMaxForDiffuse = max(motion.fdMaxHz);
        diffusePathGains = buildSharedDiffusePathGains(cfg, fdMaxForDiffuse, samplesNum);

        for iGroup = 1:numel(cfg.groups)
            group = cfg.groups{iGroup};

            if group.isRayleigh
                pathGains = diffusePathGains;
            else
                pathGains = composeMobilityAwareRicianFixedK( ...
                    cfg, diffusePathGains, pathPowersLin, motion, group.kDb);
            end

            gainDb = buildWidebandGainTraceDb(A, pathGains);

            outPath = fullfile(cfg.outputDir, group.name, sprintf('node%d.csv', nodeId));
            writeGainCsv(outPath, t, gainDb);

            out.files{end + 1} = outPath;
        end
    end

    fprintf('\n================ per-UE fading bank generation completed ================\n');
    fprintf('Output directory: %s\n', cfg.outputDir);
    fprintf('Number of UEs       : %d\n', numel(nodeIds));
    fprintf('Number of samples   : %d\n', samplesNum);
    fprintf('Groups per UE       : %d\n', numel(cfg.groups));
    fprintf('Total files         : %d\n', numel(out.files));
    fprintf('=============================================================\n\n');
end

function motion = buildMotionProfileForNode(cfg, raw, nodeId, t)
    sub = raw(raw.nodeId == nodeId, :);
    if isempty(sub)
        error('nodeId = %d was not found in the trajectory file', nodeId);
    end

    [tRaw, idx] = unique(sub.time);
    xRaw = sub.x(idx);
    yRaw = sub.y(idx);
    zRaw = sub.z(idx);
    vRaw = sub.speed(idx);
    hRaw = sub.headingRad(idx);

    [vRaw2, hRaw2] = deriveSpeedAndHeading(tRaw, xRaw, yRaw, zRaw);
    badSpeed = isnan(vRaw) | (vRaw <= 1e-9);
    badHeading = isnan(hRaw);

    vRaw(badSpeed) = vRaw2(badSpeed);
    hRaw(badHeading) = hRaw2(badHeading);

    x = interp1(tRaw, xRaw, t, 'linear', 'extrap');
    y = interp1(tRaw, yRaw, t, 'linear', 'extrap');
    z = interp1(tRaw, zRaw, t, 'linear', 'extrap');
    v = interp1(tRaw, vRaw, t, 'linear', 'extrap');
    headingRad = interp1(tRaw, hRaw, t, 'linear', 'extrap');

    bearingRad = atan2(cfg.enbPos(2) - y, cfg.enbPos(1) - x);
    relAngleRad = wrapToPiLocal(bearingRad - headingRad);

    c = 299792458;
    fdMaxHz = (v / c) * cfg.fcHz;
    fdLosHz = fdMaxHz .* cos(relAngleRad);

    motion.nodeId = nodeId;
    motion.t = t;
    motion.x = x;
    motion.y = y;
    motion.z = z;
    motion.speedMps = v;
    motion.headingRad = headingRad;
    motion.bearingRad = bearingRad;
    motion.relAngleRad = relAngleRad;
    motion.fdMaxHz = fdMaxHz;
    motion.fdLosHz = fdLosHz;
end

function [v, headingRad] = deriveSpeedAndHeading(t, x, y, z)
    n = numel(t);
    v = zeros(n, 1);
    headingRad = zeros(n, 1);

    if n < 2
        return;
    end

    dt = diff(t);
    dx = diff(x);
    dy = diff(y);
    dz = diff(z);

    dist = sqrt(dx.^2 + dy.^2 + dz.^2);
    v(2:end) = dist ./ max(dt, 1e-12);
    v(1) = v(2);

    headingRad(2:end) = atan2(dy, dx);
    headingRad(1) = headingRad(2);
end

function diffusePathGains = buildSharedDiffusePathGains(cfg, fdMaxForDiffuse, samplesNum)
    if isempty(cfg.internalSampleRateHz)
        fsChan = max(1e4, 20 * max(fdMaxForDiffuse, 1));
    else
        fsChan = cfg.internalSampleRateHz;
    end

    chan = comm.RayleighChannel( ...
        'SampleRate', fsChan, ...
        'PathDelays', cfg.pathDelaysSec, ...
        'AveragePathGains', cfg.avgPathGainsDb, ...
        'NormalizePathGains', cfg.normalizePathGains, ...
        'MaximumDopplerShift', max(fdMaxForDiffuse, eps), ...
        'FadingTechnique', 'Sum of sinusoids', ...
        'InitialTimeSource', 'Input port', ...
        'PathGainsOutputPort', true);

    nPath = numel(cfg.pathDelaysSec);
    diffusePathGains = zeros(nPath, samplesNum);

    for n = 1:samplesNum
        t0 = (n - 1) * cfg.sampleIntervalSec;
        g = getOneSnapshotPathGains(chan, t0);
        diffusePathGains(:, n) = g(:);
    end

    release(chan);
end

function ricianPathGains = composeMobilityAwareRicianFixedK(cfg, diffusePathGains, pathPowersLin, motion, kDb)
    [nPath, ~] = size(diffusePathGains);
    KLin = 10^(kDb / 10);

    dt = cfg.sampleIntervalSec;
    losPhase = 2 * pi * cumsum(motion.fdLosHz(:).') * dt;

    ricianPathGains = diffusePathGains;

    losTapIds = cfg.losClusterTapIds;
    tapOffsets = cfg.losTapPhaseOffsetsRad;


    if ~isvector(losTapIds)
        error('losClusterTapIds must be a scalar or vector');
    end
    if ~isvector(tapOffsets)
        error('losTapPhaseOffsetsRad must be a scalar or vector');
    end

    losTapIds = losTapIds(:).';
    tapOffsets = tapOffsets(:).';

    if numel(tapOffsets) ~= numel(losTapIds)
        error('losTapPhaseOffsetsRad and losClusterTapIds must have the same length');
    end

    for i = 1:numel(losTapIds)
        tapId = losTapIds(i);
        if tapId < 1 || tapId > nPath
            error('tap index in losClusterTapIds is out of range');
        end

        Pi = pathPowersLin(tapId);
        losAmp = sqrt(Pi * KLin / (KLin + 1));
        diffScale = sqrt(1 / (KLin + 1));

        ricianPathGains(tapId, :) = ...
            losAmp .* exp(1j * (losPhase + tapOffsets(i))) + ...
            diffScale .* diffusePathGains(tapId, :);
    end
end

function gainDb = buildWidebandGainTraceDb(A, pathGains)

    H = A * pathGains;


    P = mean(abs(H).^2, 1);


    P = P / mean(P);

    gainDb = 10 * log10(P + eps);
    gainDb = gainDb(:);
end

function g = getOneSnapshotPathGains(chan, t0)
    xDummy = complex(1.0, 0.0);
    [~, pathgains] = chan(xDummy, t0);
    g = reshape(pathgains(1, :), 1, []);
end

function pathPowersLin = getPathPowerLinear(cfg)
    raw = 10.^(cfg.avgPathGainsDb(:).' / 10);
    if cfg.normalizePathGains
        pathPowersLin = raw / sum(raw);
    else
        pathPowersLin = raw;
    end
end

function writeGainCsv(filePath, t, gainDb)
    fid = fopen(filePath, 'wt');
    if fid == -1
        error('Failed to create output file: %s', filePath);
    end

    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, 'time,gain_db\n');
    for i = 1:numel(t)
        fprintf(fid, '%.6f,%.12g\n', t(i), gainDb(i));
    end
end

function y = wrapToPiLocal(x)
    y = mod(x + pi, 2*pi) - pi;
end

function cfg = validateAndCompleteCfg(cfg)
    requiredFields = {
        'trajectoryCsv', 'outputDir', 'enbPos', ...
        'fcHz', 'traceDurationSec', 'sampleIntervalSec', 'numRBs', ...
        'pathDelaysSec', 'avgPathGainsDb', 'normalizePathGains', ...
        'losClusterTapIds', 'losTapPhaseOffsetsRad', ...
        'randomSeed', 'groups'
    };

    for i = 1:numel(requiredFields)
        f = requiredFields{i};
        if ~isfield(cfg, f)
            error('cfg is missing field: %s', f);
        end
    end

    if ~isfield(cfg, 'internalSampleRateHz')
        cfg.internalSampleRateHz = [];
    end

    if numel(cfg.pathDelaysSec) ~= numel(cfg.avgPathGainsDb)
        error('cfg.pathDelaysSec and cfg.avgPathGainsDb must have the same length');
    end

    if cfg.sampleIntervalSec < 1e-3
        error('Sample interval must be >= 1 ms');
    end

    ratio = cfg.sampleIntervalSec / 1e-3;
    if abs(ratio - round(ratio)) > 1e-12
        error('Sample interval must be an integer multiple of 1 ms');
    end

    samplesNumFloat = cfg.traceDurationSec / cfg.sampleIntervalSec;
    if abs(samplesNumFloat - round(samplesNumFloat)) > 1e-12
        error('traceDurationSec / sampleIntervalSec must be an integer');
    end

    if ~exist(cfg.trajectoryCsv, 'file')
        error('Trajectory file not found: %s', cfg.trajectoryCsv);
    end
end
