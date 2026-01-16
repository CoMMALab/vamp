import csv
import sys
import matplotlib.pyplot as plt

def main():
    filename = '/vamp/fetch_rrtc_results.csv'
    
    try:
        with open(filename, mode='r', newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            
            rows = []
            
            for row in reader:
                try:
                    # Convert to float for sorting, keep in row for easy access
                    row['difference_connection'] = float(row['difference_connection'])
                    row['planning_iterations'] = float(row['planning_iterations'])
                    
                    if row['planning_iterations'] > 0:
                        row['ratio'] = row['difference_connection'] / row['planning_iterations']
                    else:
                        row['ratio'] = 0.0
                        
                    rows.append(row)
                except ValueError:
                    continue
            
            # Sort by ratio descending
            rows.sort(key=lambda x: x['ratio'], reverse=True)
            
            # Plotting
            ratios = [row['ratio'] for row in rows]
            x_axis = range(1, len(rows) + 1)
            
            plt.figure(figsize=(10, 6))
            plt.plot(x_axis, ratios)
            plt.xlabel('Number of Problems')
            plt.ylabel('Ratio')
            plt.title('Ratio vs Number of Problems')
            plt.grid(True)
            plt.savefig('log/ratio_plot.png')
            print("Plot saved to ratio_plot.png")
            
            top_10 = rows[:10]
            
            if top_10:
                print(f"Top 10 problems with largest ratio (difference_connection / planning_iterations):")
                print("=" * 50)
                for i, row in enumerate(top_10, 1):
                    print(f"Rank {i}:")
                    print(f"  Problem: {row['problem']}")
                    print(f"  Problem Index: {row['problem_index']}")
                    print(f"  Ratio: {row['ratio']:.6f}")
                    print(f"  Difference Connection: {row['difference_connection']}")
                    print(f"  Planning Iterations: {row['planning_iterations']}")
                    print("-" * 30)
            else:
                print("No data found or could not parse 'difference_connection'.")
                
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
