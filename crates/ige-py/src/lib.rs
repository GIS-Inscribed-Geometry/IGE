    let opts = MicOptions {
        engine: parse_mic_engine(engine)?,
        robust_mode: parse_robust_mode(robust_mode)?,
        use_bvh: false,
    };
