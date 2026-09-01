from run_experiments import iter_trial_jobs


def test_iter_trial_jobs_pairs_policy_seeds_by_noise_and_local_trial():
    jobs = list(
        iter_trial_jobs(
            policies=["finger_center_contact_push", "pre_grasp_finger_center_correction"],
            translation_noise=[0.0, 0.005],
            trials=2,
        )
    )

    grouped = {}
    for job in jobs:
        grouped.setdefault((job.translation_noise_m, job.local_seed), set()).add(job.seed)

    assert len(jobs) == 8
    assert all(len(seeds) == 1 for seeds in grouped.values())
