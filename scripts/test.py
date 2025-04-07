import numpy as np
import matplotlib.pyplot as plt
import heapq

def minimum_operations(grid, target):
    rows, cols = grid.shape
    target_r, target_c = target

    def is_within_bounds(r, c):
        return 0 <= r < rows and 0 <= c < cols

    def is_boundary(r, c):
        return r == 0 or r == rows - 1 or c == 0 or c == cols - 1

    # Priority queue for Dijkstra's algorithm
    pq = []
    heapq.heappush(pq, (0, target_r, target_c))  # (cost, row, col)
    
    # Cost dictionary to track the minimum cost to reach each tile
    cost = {(target_r, target_c): 0}
    
    while pq:
        current_cost, r, c = heapq.heappop(pq)
        
        # If we've reached a boundary tile, return the cost
        if is_boundary(r, c):
            return current_cost *2 +1
        
        # Explore neighbors
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if is_within_bounds(nr, nc):
                # Compute the cost to move the blocking tile
                new_cost = current_cost + (1 if grid[nr, nc] == 1 else 0)
                
                # Update if we found a cheaper way to reach this tile
                if (nr, nc) not in cost or new_cost < cost[(nr, nc)]:
                    cost[(nr, nc)] = new_cost
                    heapq.heappush(pq, (new_cost, nr, nc))
    
    return -1  # If no path to boundary exists



if __name__ == "__main__":
    sc5 = 7/5 + 4
    sc6 = 8/6 + 4 + 1/6 * (sc5 + 1) 
    sc7 = 9/7 + 6
    print("sc5=", sc5)
    print("sc6=", sc6)
    print("sc7=", sc7)
    p = 1/4 *(3 + sc7) + 2/4 * (1 + 1 + sc6) + 1/4 * (1 + 2 + sc5)
    print(f"p={p}")
    p8 = 3/4*(4/7 + 2/7 * 4/6 + 2/7* 1/6 * 4/5)
    print(p8)
    p10 = 1/4 * 6/7 + 3/4 * (1/7 + 2/7 * 1/6 + 2/7 * 1/6 * 1/5)
    print(p10)
    p12 = 1/4 * 1/7
    print(p12)
    
    mean = p8 * 8 + p10 * 10 + p12 * 12
    print("mean=", mean)

    width = 4
    height = 3
    trials = [10, 100, 1000, 10000, 25000, 50000, 75000, 100000]
    # trials = [100]
    # trials = [100, 1000, 10000]

    mean_operations = [] 
    fig, ax = plt.subplots()
    for i in range(len(trials)):
        trial = trials[i]
        total_operations = []
        operations_dict = {}
        for _ in range(trial):
            shelf = np.ones((height, width))
            order = np.arange(width * height)
            np.random.shuffle(order)
            order_dict = {order[i]: (i//width, i%width) for i in range(width * height)}
            total = 0
            for i in range(width * height):
                total += minimum_operations(shelf, order_dict[i])
                shelf[order_dict[i]] = 0
            if total in operations_dict:
                operations_dict[total] += 1
            else:
                operations_dict[total] = 1
            total_operations.append(total)
        # sort the dictionary by keys
        total_freq = sum(operations_dict.values())
        operations_dict = dict(sorted(operations_dict.items()))
        operations_dict = {k: v/total_freq for k, v in operations_dict.items()}
        ax.plot(list(operations_dict.keys()), list(operations_dict.values()), 'o-', label=f'{trial} trials')
        mean_operations.append(np.mean(total_operations)) 
    
    print(mean_operations)
    print(operations_dict)
    plt.legend()
    plt.show()
    # plt.plot(trials, mean_operations, '-o')
    # plt.xlabel('Number of trials')
    # plt.ylabel('Mean number of operations')
    # plt.show()