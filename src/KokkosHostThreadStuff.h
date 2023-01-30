#pragma once

template <class ExecSpace>
struct SpaceInstance
{
    static ExecSpace create() { return ExecSpace(); }
    static void destroy(ExecSpace &) {}
    static bool overlap() { return false; }
};

#ifndef KOKKOS_ENABLE_DEBUG
#ifdef KOKKOS_ENABLE_CUDA
template <>
struct SpaceInstance<Kokkos::Cuda>
{
    static Kokkos::Cuda create()
    {
        cudaStream_t stream;
        cudaStreamCreate(&stream);
        return Kokkos::Cuda(stream);
    }
    static void destroy(Kokkos::Cuda &space)
    {
        cudaStream_t stream = space.cuda_stream();
        cudaStreamDestroy(stream);
    }
    static bool overlap()
    {
        bool value = true;
        auto local_rank_str = std::getenv("CUDA_LAUNCH_BLOCKING");
        if (local_rank_str)
        {
            value = (std::stoi(local_rank_str) == 0);
        }
        return value;
    }
};
#endif
#endif