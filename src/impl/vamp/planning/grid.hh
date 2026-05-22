#pragma once

// grid.hh — Compile-time Grid (replaces GridB/GridN/Grid from OMPL)
//
// Maps a D-dimensional integer coordinate to a Cell that holds:
//   • the list of motion indices in the pool that landed in this cell
//   • KPIECE book-keeping (score, coverage, selections, iteration, importance)
//
// "Internal" cells are those whose D-dimensional face-neighbours all exist in
// the grid.  "External" (border) cells are the rest.  We maintain two lazy
// max-heaps ordered by `importance` so that topInternal() / topExternal() are
// O(log n) amortised.
//
// All random number generation uses a vamp::rng::Distribution reference passed
// in by the caller (typically rng->dist), so there is no RNG state inside the
// Grid itself.

#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include <vamp/random/rng.hh>

namespace vamp::planning::detail
{

template <std::size_t D>
struct GridCoordHash
{
    std::size_t operator()(const std::array<int, D> &c) const noexcept
    {
        std::size_t seed = 0;
        for (int v : c)
            seed ^= std::hash<int>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

template <std::size_t D>
struct CellData
{
    std::vector<std::size_t> motion_indices;
    double   coverage{1.0};
    unsigned selections{1};
    double   score{1.0};
    unsigned iteration{0};
    double   importance{0.0};
    std::size_t neighbors{0};
    bool     is_external{true};
    unsigned heap_gen{0};  // bumped on every importance update (lazy-heap token)
};

template <std::size_t D>
struct HeapEntry
{
    double             importance;
    std::array<int, D> coord;
    unsigned           gen;
};

template <std::size_t D>
struct HeapCmp
{
    bool operator()(const HeapEntry<D> &a, const HeapEntry<D> &b) const noexcept
    {
        return a.importance < b.importance;  // max-heap
    }
};

template <typename T, typename Cmp>
struct LazyHeap
{
    std::vector<T> data;
    Cmp            cmp;

    void push(T t)
    {
        data.push_back(std::move(t));
        std::push_heap(data.begin(), data.end(), cmp);
    }

    void pop()
    {
        std::pop_heap(data.begin(), data.end(), cmp);
        data.pop_back();
    }

    bool empty() const noexcept { return data.empty(); }
};

template <std::size_t D>
class Grid
{
public:
    using Coord = std::array<int, D>;
    using Cell  = CellData<D>;
    using Map   = std::unordered_map<Coord, Cell, GridCoordHash<D>>;

    // ── Access ────────────────────────────────────────────────────────────

    Cell *getCell(const Coord &c) noexcept
    {
        auto it = cells_.find(c);
        return (it == cells_.end()) ? nullptr : &it->second;
    }

    // Create a new cell, update neighbour bookkeeping, push to the appropriate
    // heap.  Returns a stable pointer to the new cell.
    Cell *createCell(const Coord &c, unsigned iteration, double initial_score)
    {
        Cell cell;
        cell.iteration   = iteration;
        cell.score       = initial_score;
        cell.is_external = true;

        auto [it, ok] = cells_.emplace(c, std::move(cell));
        (void)ok;
        Cell *nc = &it->second;

        static constexpr int dirs[2] = {-1, +1};
        for (std::size_t dim = 0; dim < D; ++dim)
        {
            for (int dir : dirs)
            {
                Coord nb  = c;
                nb[dim]  += dir;
                auto nit  = cells_.find(nb);
                if (nit != cells_.end())
                {
                    ++nc->neighbors;
                    ++nit->second.neighbors;
                    recomputeExternal(nit->second);
                    pushToHeap(nit->second, nb);
                }
            }
        }

        ++externalCount_;          // new cell starts external
        recomputeExternal(*nc);    // may decrement immediately for D==0
        return nc;
    }

    void updateImportance(Cell *cell, const Coord &c)
    {
        cell->importance =
            cell->score /
            (static_cast<double>(cell->neighbors + 1) * cell->coverage * cell->selections);
        ++cell->heap_gen;
        pushToHeap(*cell, c);
    }

    // ── Selection ─────────────────────────────────────────────────────────
    //
    // All randomness goes through `dist` — no internal RNG state.
    //
    // Returns the coordinate of the selected cell, or nullopt if the grid is
    // empty.
    std::optional<Coord> selectCell(float border_fraction, vamp::rng::Distribution &dist)
    {
        pruneHeaps();
        if (cells_.empty())
            return std::nullopt;

        const float frac_ext =
            externalCount_ == 0
                ? 0.0F
                : static_cast<float>(externalCount_) / static_cast<float>(cells_.size());

        const bool pick_external =
            dist.uniform_01() < std::max(border_fraction, frac_ext);

        if (pick_external)
        {
            while (!ext_heap_.empty())
            {
                const auto &e = ext_heap_.data.front();
                Cell       *cell = getCell(e.coord);
                if (cell && cell->heap_gen == e.gen && cell->is_external)
                    return e.coord;
                ext_heap_.pop();
            }
        }

        // Fall back to internal heap (or any surviving external entry)
        while (!int_heap_.empty())
        {
            const auto &e = int_heap_.data.front();
            Cell       *cell = getCell(e.coord);
            if (cell && cell->heap_gen == e.gen && !cell->is_external)
                return e.coord;
            int_heap_.pop();
        }

        // Last resort: any remaining cell
        return cells_.begin()->first;
    }

    std::size_t size()   const noexcept { return cells_.size(); }
    Map        &cells()        noexcept { return cells_; }
    const Map  &cells() const  noexcept { return cells_; }

private:
    void recomputeExternal(Cell &cell) noexcept
    {
        const bool now_ext = (cell.neighbors < 2 * D);
        if (cell.is_external && !now_ext)
            --externalCount_;
        else if (!cell.is_external && now_ext)
            ++externalCount_;
        cell.is_external = now_ext;
    }

    void pushToHeap(Cell &cell, const Coord &c)
    {
        HeapEntry<D> e{cell.importance, c, cell.heap_gen};
        if (cell.is_external)
            ext_heap_.push(e);
        else
            int_heap_.push(e);
    }

    void pruneHeaps()
    {
        auto prune = [&](auto &heap)
        {
            while (!heap.empty())
            {
                const auto &e = heap.data.front();
                Cell       *cell = getCell(e.coord);
                if (cell && cell->heap_gen == e.gen)
                    return;
                heap.pop();
            }
        };
        prune(ext_heap_);
        prune(int_heap_);
    }

    Map             cells_;
    std::size_t     externalCount_{0};
    LazyHeap<HeapEntry<D>, HeapCmp<D>> ext_heap_;
    LazyHeap<HeapEntry<D>, HeapCmp<D>> int_heap_;
};

}  // namespace vamp::planning::detail