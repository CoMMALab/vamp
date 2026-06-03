#pragma once

// grid.hh — Compile-time Grid (replaces GridB/GridN/Grid from OMPL)
//
// Maps a D-dimensional integer coordinate to a Cell that holds:
//   • the list of motion indices in the pool that landed in this cell
//   • KPIECE book-keeping (score, coverage, selections, iteration, importance)
//
// "Internal" cells are those whose D-dimensional face-neighbours all exist in
// the grid.  "External" (border) cells are the rest.  We maintain two
// updatable max-heaps ordered by `importance` so that the best internal /
// external cell can be found in O(1) and updated in O(log n).
//
// All random number generation uses a vamp::rng::Distribution reference passed
// in by the caller (typically rng->dist), so there is no RNG state inside the
// Grid itself.

#include <array>
#include <cassert>
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include <cmath>
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

// ── Updatable Binary Heap ──────────────────────────────────────────────────
//
// A max-heap where each inserted element receives a stable handle (Element*)
// that supports O(log n) priority updates and O(log n) removal.  Modelled
// after OMPL's BinaryHeap but simplified (no event callbacks).
//
// Cmp(a, b) must return true when a has *higher* priority than b (i.e. a
// should be closer to the top).  This is the opposite convention from
// std::less / std::push_heap.

template <typename T, typename Cmp>
class UpdatableHeap
{
public:
    struct Element
    {
        friend class UpdatableHeap;

    private:
        Element() = default;
        ~Element() = default;
        unsigned int position{0};  // index in vector_

    public:
        T data;
    };

    UpdatableHeap() = default;
    explicit UpdatableHeap(Cmp cmp) : cmp_(std::move(cmp)) {}

    ~UpdatableHeap() { clear(); }

    UpdatableHeap(const UpdatableHeap &) = delete;
    UpdatableHeap &operator=(const UpdatableHeap &) = delete;

    UpdatableHeap(UpdatableHeap &&other) noexcept
        : vector_(std::move(other.vector_)), cmp_(std::move(other.cmp_))
    {
    }

    UpdatableHeap &operator=(UpdatableHeap &&other) noexcept
    {
        if (this != &other)
        {
            clear();
            vector_ = std::move(other.vector_);
            cmp_ = std::move(other.cmp_);
        }
        return *this;
    }

    void clear()
    {
        for (auto *e : vector_)
            delete e;
        vector_.clear();
    }

    // Insert a new element.  Returns a handle that can be used with
    // update() and remove().
    Element *insert(const T &data)
    {
        auto *e = new Element();
        e->data = data;
        e->position = static_cast<unsigned int>(vector_.size());
        vector_.push_back(e);
        percolateUp(e->position);
        return e;
    }

    // Re-position an element after its priority has changed.
    void update(Element *e)
    {
        assert(e->position < vector_.size());
        assert(vector_[e->position] == e);
        percolateUp(e->position);
        percolateDown(e->position);
    }

    // Remove an element from the heap and free it.
    void remove(Element *e)
    {
        removePos(e->position);
    }

    // Peek at the top (highest-priority) element.  Returns nullptr if empty.
    Element *top() const
    {
        return vector_.empty() ? nullptr : vector_[0];
    }

    // Remove the top element.
    void pop()
    {
        if (!vector_.empty())
            removePos(0);
    }

    bool empty() const noexcept { return vector_.empty(); }
    unsigned int size() const noexcept { return static_cast<unsigned int>(vector_.size()); }

private:
    std::vector<Element *> vector_;
    Cmp cmp_;

    // Move element at pos up while it has higher priority than its parent.
    void percolateUp(unsigned int pos)
    {
        Element *tmp = vector_[pos];
        unsigned int child = pos;
        unsigned int parent = (pos - 1) >> 1;

        while (child > 0 && cmp_(tmp->data, vector_[parent]->data))
        {
            vector_[child] = vector_[parent];
            vector_[child]->position = child;
            child = parent;
            parent = (parent - 1) >> 1;
        }
        if (child != pos)
        {
            vector_[child] = tmp;
            vector_[child]->position = child;
        }
    }

    // Move element at pos down while a child has higher priority.
    void percolateDown(unsigned int pos)
    {
        const unsigned int n = static_cast<unsigned int>(vector_.size());
        Element *tmp = vector_[pos];
        unsigned int parent = pos;
        unsigned int child = (pos + 1) << 1;  // right child

        while (child < n)
        {
            // Pick the child with higher priority (cmp_ returns true for higher).
            // cmp_(left, right) true → left has higher priority → pick left (child-1).
            if (cmp_(vector_[child - 1]->data, vector_[child]->data))
                --child;
            // child is now the better child
            if (cmp_(vector_[child]->data, tmp->data))
            {
                vector_[parent] = vector_[child];
                vector_[parent]->position = parent;
            }
            else
                break;
            parent = child;
            child = (child + 1) << 1;
        }
        if (child == n)
        {
            --child;  // left child only
            if (cmp_(vector_[child]->data, tmp->data))
            {
                vector_[parent] = vector_[child];
                vector_[parent]->position = parent;
                parent = child;
            }
        }
        if (parent != pos)
        {
            vector_[parent] = tmp;
            vector_[parent]->position = parent;
        }
    }

    void removePos(unsigned int pos)
    {
        const int n = static_cast<int>(vector_.size()) - 1;
        delete vector_[pos];
        if (static_cast<int>(pos) < n)
        {
            vector_[pos] = vector_.back();
            vector_[pos]->position = pos;
            vector_.pop_back();
            percolateUp(pos);
            percolateDown(pos);
        }
        else
        {
            vector_.pop_back();
        }
    }
};

// ── Heap payload & comparator ──────────────────────────────────────────────

template <std::size_t D>
struct HeapEntry
{
    double             importance;
    std::array<int, D> coord;
};

template <std::size_t D>
struct HeapCmp
{
    bool operator()(const HeapEntry<D> &a, const HeapEntry<D> &b) const noexcept
    {
        return a.importance > b.importance;  // max-heap: higher importance on top
    }
};

// ── Cell data ──────────────────────────────────────────────────────────────

template <std::size_t D>
class Grid;  // forward declaration for friend access

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

    // Updatable-heap handle (owned by one of the two heaps in Grid).
    // nullptr when the cell is not currently in any heap.
    using Heap = UpdatableHeap<HeapEntry<D>, HeapCmp<D>>;
    typename Heap::Element *heap_handle{nullptr};
    bool in_ext_heap{false};  // which heap the handle belongs to
};

// ── Grid ───────────────────────────────────────────────────────────────────

template <std::size_t D>
class Grid
{
public:
    using Coord = std::array<int, D>;
    using Cell  = CellData<D>;
    using Map   = std::unordered_map<Coord, Cell, GridCoordHash<D>>;
    using Heap  = typename Cell::Heap;

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
                    recomputeExternal(nit->second, nb);
                    ensureInHeap(nit->second, nb);
                }
            }
        }

        ++externalCount_;          // new cell starts external
        recomputeExternal(*nc, c); // may decrement immediately for D==0
        ensureInHeap(*nc, c);
        return nc;
    }

    void updateImportance(Cell *cell, const Coord &c)
    {
        cell->importance =
            cell->score /
            (static_cast<double>(cell->neighbors + 1) * cell->coverage * cell->selections);
        ensureInHeap(*cell, c);
    }

    // Boost every cell's score by (1 + log(iteration)) and recompute all
    // importances.  Called when the selected cell's score has underflowed to
    // near zero, mirroring OMPL's Discretization::selectMotion recovery block.
    void rescoreAllCells(unsigned int iteration)
    {
        const double boost = 1.0 + std::log(static_cast<double>(iteration));
        for (auto &[coord, cell] : cells_)
        {
            cell.score += boost;
            cell.importance =
                cell.score /
                (static_cast<double>(cell.neighbors + 1) * cell.coverage * cell.selections);
            updateHeapElement(cell);
        }
    }

    // ── Selection ─────────────────────────────────────────────────────────
    //
    // All randomness goes through `dist` — no internal RNG state.
    //
    // Returns the coordinate of the selected cell, or nullopt if the grid is
    // empty.
    std::optional<Coord> selectCell(float border_fraction, vamp::rng::Distribution &dist)
    {
        if (cells_.empty())
            return std::nullopt;

        const float frac_ext =
            externalCount_ == 0
                ? 0.0F
                : static_cast<float>(externalCount_) / static_cast<float>(cells_.size());

        const bool pick_external =
            dist.uniform_01() < std::max(border_fraction, frac_ext);

        if (pick_external && !ext_heap_.empty())
        {
            return ext_heap_.top()->data.coord;
        }

        // Fall back to internal heap
        if (!int_heap_.empty())
        {
            return int_heap_.top()->data.coord;
        }

        // Last resort: any remaining cell (e.g. if ext_heap had elements but
        // was skipped because pick_external was false and int_heap is empty).
        if (!ext_heap_.empty())
        {
            return ext_heap_.top()->data.coord;
        }

        return cells_.begin()->first;
    }

    std::size_t size()   const noexcept { return cells_.size(); }
    Map        &cells()        noexcept { return cells_; }
    const Map  &cells() const  noexcept { return cells_; }

private:
    // Recompute whether a cell is external based on its neighbor count.
    // If the status changes, move the cell between heaps.
    void recomputeExternal(Cell &cell, const Coord &c) noexcept
    {
        const bool now_ext = (cell.neighbors < 2 * D);
        if (cell.is_external == now_ext)
            return;  // no change

        if (cell.is_external && !now_ext)
            --externalCount_;
        else if (!cell.is_external && now_ext)
            ++externalCount_;
        cell.is_external = now_ext;

        // Move between heaps if the cell is currently in one.
        if (cell.heap_handle)
        {
            if (cell.in_ext_heap)
            {
                ext_heap_.remove(cell.heap_handle);
                cell.heap_handle = nullptr;
            }
            else
            {
                int_heap_.remove(cell.heap_handle);
                cell.heap_handle = nullptr;
            }
            // Re-insert into the correct heap.
            insertIntoHeap(cell, c);
        }
    }

    // Update the heap element's importance value in-place (no insertion/removal).
    void updateHeapElement(Cell &cell)
    {
        if (cell.heap_handle)
        {
            cell.heap_handle->data.importance = cell.importance;
            (cell.in_ext_heap ? ext_heap_ : int_heap_).update(cell.heap_handle);
        }
    }

    // Ensure a cell is present in the correct heap with up-to-date importance.
    void ensureInHeap(Cell &cell, const Coord &c)
    {
        if (cell.heap_handle)
        {
            // Already in a heap — just update priority.
            cell.heap_handle->data.importance = cell.importance;
            (cell.in_ext_heap ? ext_heap_ : int_heap_).update(cell.heap_handle);
        }
        else
        {
            insertIntoHeap(cell, c);
        }
    }

    // Insert a cell into the appropriate heap (must not already be in a heap).
    void insertIntoHeap(Cell &cell, const Coord &c)
    {
        HeapEntry<D> entry{cell.importance, c};
        if (cell.is_external)
        {
            cell.heap_handle = ext_heap_.insert(entry);
            cell.in_ext_heap = true;
        }
        else
        {
            cell.heap_handle = int_heap_.insert(entry);
            cell.in_ext_heap = false;
        }
    }

    Map             cells_;
    std::size_t     externalCount_{0};
    Heap            ext_heap_;
    Heap            int_heap_;
};

}  // namespace vamp::planning::detail