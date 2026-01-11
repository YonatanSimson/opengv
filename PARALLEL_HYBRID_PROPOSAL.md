# Parallel Hybrid SQPnP Implementation Proposal

## Problem Statement

The current hybrid implementation runs algorithms **sequentially**:
```
Total Time = Time_SQPnP + Time_EPnP  (2× overhead if algorithms are equal speed)
```

If timing is critical, 2× overhead may be unacceptable for real-time applications.

---

## Solution: Parallel Execution

Run both algorithms simultaneously using multithreading:
```
Total Time = max(Time_SQPnP, Time_EPnP)  (Near-zero overhead if balanced!)
```

### Why This Works

1. **Independent Computations**: SQPnP and EPnP don't share mutable state
2. **Modern Hardware**: Most CPUs have 4+ cores (2 cores easily available)
3. **Near-Zero Overhead**: Wall-clock time ≈ slower algorithm only
4. **Simple Implementation**: C++11 `std::thread` or `std::async`

---

## Performance Analysis

### Theoretical Speedup

Assume:
- `T_sqpnp = 10ms` (SQPnP computation time)
- `T_epnp = 8ms` (EPnP computation time)
- `T_select = 0.1ms` (Selection overhead)

**Sequential (current):**
```
T_sequential = T_sqpnp + T_epnp + T_select = 10 + 8 + 0.1 = 18.1ms
```

**Parallel (proposed):**
```
T_parallel = max(T_sqpnp, T_epnp) + T_select = max(10, 8) + 0.1 = 10.1ms
```

**Speedup: 1.79× (44% faster!)**

### Worst-Case Scenarios

Even if algorithms take the same time:
```
T_sqpnp = T_epnp = 10ms

Sequential: 20ms
Parallel:   10ms

Speedup: 2× (50% faster)
```

### Best-Case Scenarios

If one algorithm dominates:
```
T_sqpnp = 15ms, T_epnp = 2ms

Sequential: 17ms
Parallel:   15ms

Speedup: 1.13× (still an improvement)
```

---

## Implementation Options

### Option 1: std::async (Recommended) ✅

**Pros:**
- Simplest implementation
- Automatic thread management
- Exception-safe
- C++11 standard

**Cons:**
- Slight overhead from std::future
- May spawn threads unnecessarily

```cpp
#include <future>

double Sqpnp::compute_pose(double R[3][3], double t[3])
{
    // Launch SQPnP in parallel
    auto sqpnp_future = std::async(std::launch::async, [this]() {
        Eigen::MatrixXd Omega;
        compute_omega_matrix(Omega);
        Eigen::Matrix3d R_sqp;
        Eigen::Vector3d t_sqp;
        double error = sqp_solve(Omega, R_sqp, t_sqp);

        // Return result
        struct Result { Eigen::Matrix3d R; Eigen::Vector3d t; double error; };
        return Result{R_sqp, t_sqp, error};
    });

    // Run EPnP in current thread
    choose_control_points();
    compute_barycentric_coordinates();
    // ... EPnP computation ...
    double epnp_best_error = /*...*/;

    // Wait for SQPnP to finish
    auto sqpnp_result = sqpnp_future.get();

    // Select best
    if(sqpnp_result.error < epnp_best_error)
        return copy_and_return(sqpnp_result);
    else
        return copy_and_return(epnp_result);
}
```

### Option 2: Thread Pool (Advanced)

**Pros:**
- No thread creation overhead
- Better for repeated calls
- More control over scheduling

**Cons:**
- More complex implementation
- Requires external dependency or custom pool

```cpp
// Using a global thread pool
ThreadPool pool(2);  // 2 threads

double Sqpnp::compute_pose(double R[3][3], double t[3])
{
    auto sqpnp_task = pool.enqueue([this]() { /* SQPnP work */ });
    auto epnp_task = pool.enqueue([this]() { /* EPnP work */ });

    auto sqpnp_result = sqpnp_task.get();
    auto epnp_result = epnp_task.get();

    return select_best(sqpnp_result, epnp_result);
}
```

### Option 3: Conditional Parallelism

**Pros:**
- Flexibility for different use cases
- Can disable for embedded systems

**Cons:**
- More code paths to test

```cpp
enum SQPnPMode {
    FAST,         // EPnP only
    PARALLEL,     // Run both in parallel (default)
    SEQUENTIAL,   // Run both sequentially
    ACCURATE      // Run all variations
};

double Sqpnp::compute_pose(
    double R[3][3],
    double t[3],
    SQPnPMode mode = PARALLEL)
{
    switch(mode) {
        case FAST:
            return compute_epnp_only(R, t);
        case PARALLEL:
            return compute_parallel_hybrid(R, t);
        case SEQUENTIAL:
            return compute_sequential_hybrid(R, t);
        case ACCURATE:
            return compute_all_variations(R, t);
    }
}
```

---

## Recommended Implementation

### Phase 1: Add Parallel Option

1. Keep current sequential implementation as default
2. Add new `compute_pose_parallel()` method
3. Expose via API for users who want performance

```cpp
// Public API addition
namespace opengv {
namespace absolute_pose {
    // Existing (sequential hybrid)
    transformation_t sqpnp(const AbsoluteAdapterBase& adapter);

    // New (parallel hybrid)
    transformation_t sqpnp_parallel(const AbsoluteAdapterBase& adapter);
}
}
```

### Phase 2: Make Parallel Default (After Testing)

After validating parallel version:
1. Switch default to parallel
2. Keep sequential as fallback option
3. Document performance characteristics

---

## Thread Safety Considerations

### Current Code Analysis

Looking at `Sqpnp.cpp`, the class has **mutable state**:
```cpp
class Sqpnp {
    double * pws;    // World points
    double * us;     // Bearing vectors
    double * alphas; // Barycentric coordinates
    double * pcs;    // Camera points
    int * signs;     // Sign indicators
    double cws[4][3], ccs[4][3];  // Control points
    int number_of_correspondences;
};
```

**Challenge**: EPnP uses `cws`, `ccs`, `alphas`, `pcs` which SQPnP doesn't need.

### Solution Approaches

**Approach A: Thread-Local Copies (Safest)**
```cpp
// Create separate instances
Sqpnp sqpnp_solver;
Sqpnp epnp_solver;

// Copy input data
sqpnp_solver.copy_correspondences_from(this);
epnp_solver.copy_correspondences_from(this);

// Run in parallel - each has own state
auto sqpnp_future = std::async([&]() { return sqpnp_solver.compute_sqpnp(); });
auto epnp_result = epnp_solver.compute_epnp();
auto sqpnp_result = sqpnp_future.get();
```

**Approach B: Refactor for Immutability (Better Long-term)**
```cpp
// Make algorithms functional (no mutable state)
struct PoseResult {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double error;
};

PoseResult compute_sqpnp(
    const double* pws,
    const double* us,
    int n);

PoseResult compute_epnp(
    const double* pws,
    const double* us,
    int n);
```

---

## Performance Validation

### Benchmark Test

```cpp
// Compare sequential vs parallel
for(int n_points : {10, 50, 100, 200}) {
    auto data = generate_test_data(n_points);

    // Sequential
    auto t1 = now();
    auto result_seq = sqpnp_sequential(data);
    auto t2 = now();
    double time_seq = duration(t1, t2);

    // Parallel
    auto t3 = now();
    auto result_par = sqpnp_parallel(data);
    auto t4 = now();
    double time_par = duration(t3, t4);

    cout << "n=" << n_points
         << " Sequential: " << time_seq << "ms"
         << " Parallel: " << time_par << "ms"
         << " Speedup: " << (time_seq/time_par) << "×" << endl;
}
```

**Expected Results:**
```
n=10   Sequential: 0.8ms  Parallel: 0.5ms  Speedup: 1.6×
n=50   Sequential: 2.1ms  Parallel: 1.2ms  Speedup: 1.75×
n=100  Sequential: 3.5ms  Parallel: 2.0ms  Speedup: 1.75×
n=200  Sequential: 6.2ms  Parallel: 3.5ms  Speedup: 1.77×
```

---

## Cost-Benefit Analysis

### Costs

1. **Implementation Complexity**: Moderate
   - ~100 lines of additional code
   - Thread management overhead
   - Testing both paths

2. **Memory Overhead**: Minimal
   - ~2× memory during computation
   - Temporary (released after selection)
   - Negligible on modern systems

3. **CPU Utilization**: Higher
   - Uses 2 cores instead of 1
   - May impact other processes
   - Configurable/disableable

### Benefits

1. **Performance**: **1.5-2× faster** wall-clock time
2. **Real-time Viability**: Enables 60+ FPS tracking
3. **Better Robustness**: Still runs both algorithms
4. **Scalability**: Works with any hardware (auto-adapts)

### Decision Matrix

| Use Case | Sequential | Parallel | Recommendation |
|----------|-----------|----------|----------------|
| **Offline processing** | ✓ | ✓ | Either (sequential simpler) |
| **Real-time tracking** | ✗ | ✓ | **Parallel required** |
| **Embedded (1 core)** | ✓ | ✗ | Sequential only |
| **Desktop/Server** | ✓ | ✓✓ | **Parallel preferred** |
| **Mobile (4+ cores)** | ✓ | ✓✓ | **Parallel preferred** |

---

## Implementation Roadmap

### Milestone 1: Proof of Concept (1 day)
- [ ] Implement `compute_pose_parallel()` with std::async
- [ ] Add basic benchmark comparison
- [ ] Verify correctness (results match sequential)

### Milestone 2: Production Ready (3 days)
- [ ] Handle all edge cases (small n, degenerate cases)
- [ ] Add comprehensive tests
- [ ] Document thread safety guarantees
- [ ] Performance validation across platforms

### Milestone 3: Optimization (2 days)
- [ ] Profile and optimize hot paths
- [ ] Consider thread pool for repeated calls
- [ ] Add configuration options
- [ ] Benchmark on embedded platforms

### Total: ~1 week of focused development

---

## Code Example: Minimal Parallel Implementation

```cpp
// In Sqpnp.hpp
class Sqpnp {
public:
    double compute_pose(double R[3][3], double t[3]);
    double compute_pose_parallel(double R[3][3], double t[3]);  // NEW

private:
    struct PoseResult {
        double R[3][3];
        double t[3];
        double error;
    };

    PoseResult compute_sqpnp_internal();
    PoseResult compute_epnp_internal();
};

// In Sqpnp.cpp
#include <future>

double Sqpnp::compute_pose_parallel(double R[3][3], double t[3])
{
    // Create copies for thread safety
    Sqpnp sqpnp_copy = *this;  // Copy constructor
    Sqpnp epnp_copy = *this;

    // Launch SQPnP in background thread
    auto sqpnp_future = std::async(std::launch::async,
        &Sqpnp::compute_sqpnp_internal, &sqpnp_copy);

    // Run EPnP in current thread
    PoseResult epnp_result = epnp_copy.compute_epnp_internal();

    // Wait for SQPnP to complete
    PoseResult sqpnp_result = sqpnp_future.get();

    // Select best result
    if(sqpnp_result.error < epnp_result.error) {
        copy_R_and_t(sqpnp_result.R, sqpnp_result.t, R, t);
        return sqpnp_result.error;
    } else {
        copy_R_and_t(epnp_result.R, epnp_result.t, R, t);
        return epnp_result.error;
    }
}
```

---

## Conclusion

### Is Parallel Hybrid Worth Implementing?

**YES** ✅✅✅

**Key Reasons:**
1. **Eliminates timing concern** - No longer 2× slower
2. **Simple to implement** - ~100 lines with std::async
3. **Backward compatible** - Keep sequential as option
4. **Proven approach** - Used in production systems
5. **Modern hardware** - Multi-core is standard

### Recommendation

**Implement parallel hybrid as the new default**, with fallback to sequential for:
- Single-core embedded systems
- Platforms without threading support
- User preference (via configuration)

**Expected Impact:**
- Real-time tracking: **Enabled** ✅
- Desktop applications: **1.5-2× faster** ✅
- Embedded systems: **No change** (use sequential) ✅
- Code complexity: **Minimal increase** ✅

---

**Priority**: **HIGH** - This directly addresses the performance concern while maintaining robustness.

**Effort**: **LOW-MEDIUM** - ~1 week of focused development

**Value**: **VERY HIGH** - Enables real-time use cases while keeping hybrid benefits

