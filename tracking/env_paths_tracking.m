function p = env_paths_tracking(p)

    p.net_base_path = './';
    p.seq_base_path = '../demo-sequences';
    p.seq_vot_base_path = []; %'/path/to/VOT/evaluation/sequences/'; % (optional)
    p.stats_path = []; %'/path/to/ILSVRC15-VID/stats.mat'; % (optional)

end
