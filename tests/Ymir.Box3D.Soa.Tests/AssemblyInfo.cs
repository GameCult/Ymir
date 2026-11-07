// Box3D world creation is not safe to run concurrently in one process: two threads calling
// b3CreateWorld abort the host (found when two test classes created worlds at once). Tests that create
// worlds run one at a time.
[assembly: CollectionBehavior(DisableTestParallelization = true)]
