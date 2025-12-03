#include "../../ds/headers.hpp"

/**
 * @param arr: the array to be sorted
 * @param left: starting index of left subarray
 * @param mid: ending index of left subarray
 * @param right: ending index of right subarray
 */
void merge(vector<int>& arr, int left, int mid, int right) {
    // Create temporary arrays for the two subarrays
    int n1 = mid - left + 1;  // Size of left subarray
    int n2 = right - mid;     // Size of right subarray
    
    vector<int> leftArr(n1);
    vector<int> rightArr(n2);
    
    // Copy data to temporary arrays
    for (int i = 0; i < n1; i++)
        leftArr[i] = arr[left + i];
    for (int j = 0; j < n2; j++)
        rightArr[j] = arr[mid + 1 + j];
    
    // Merge the temporary arrays back into arr[left..right]
    int i = 0;    // Initial index of left subarray
    int j = 0;    // Initial index of right subarray
    int k = left; // Initial index of merged subarray
    
    while (i < n1 && j < n2) {
        if (leftArr[i] <= rightArr[j]) {
            arr[k] = leftArr[i];
            i++;
        } else {
            arr[k] = rightArr[j];
            j++;
        }
        k++;
    }
    
    // Copy the remaining elements of leftArr[], if any
    while (i < n1) {
        arr[k] = leftArr[i];
        i++;
        k++;
    }
    
    // Copy the remaining elements of rightArr[], if any
    while (j < n2) {
        arr[k] = rightArr[j];
        j++;
        k++;
    }
}

void mergeSort(vector<int>& arr, int left, int right) {
    if (left >= right) {
        return; // Base case: array with 0 or 1 element is already sorted
    }
    
    // Find the middle point to divide the array into two halves
    int mid = left + (right - left) / 2;
    
    // Recursively sort both halves
    mergeSort(arr, left, mid);
    mergeSort(arr, mid + 1, right);
    
    // Merge the sorted halves
    merge(arr, left, mid, right);
}

void mergeSort(vector<int>& arr) {
    if (arr.size() <= 1) return;
    mergeSort(arr, 0, arr.size() - 1);
}

bool isSorted(const vector<int>& arr) {
    for (size_t i = 1; i < arr.size(); i++) {
        if (arr[i] < arr[i - 1]) {
            return false;
        }
    }
    return true;
}

void printArray(const string& label, const vector<int>& arr) {
    cout << label << ": " << arr << endl;
}

int main() {
    cout << "=== Recursive Merge Sort Implementation ===" << endl << endl;
    
    // Test case 1: Random unsorted array
    vector<int> arr1 = {64, 34, 25, 12, 22, 11, 90, 5};
    cout << "Test Case 1: Random Array" << endl;
    printArray("Original", arr1);
    mergeSort(arr1);
    printArray("Sorted  ", arr1);
    cout << "Is sorted: ";
    print_bool(isSorted(arr1));
    cout << endl;
    
    // Test case 2: Already sorted array
    vector<int> arr2 = {1, 2, 3, 4, 5, 6, 7, 8};
    cout << "Test Case 2: Already Sorted Array" << endl;
    printArray("Original", arr2);
    mergeSort(arr2);
    printArray("Sorted  ", arr2);
    cout << "Is sorted: ";
    print_bool(isSorted(arr2));
    cout << endl;
    
    // Test case 3: Reverse sorted array
    vector<int> arr3 = {8, 7, 6, 5, 4, 3, 2, 1};
    cout << "Test Case 3: Reverse Sorted Array" << endl;
    printArray("Original", arr3);
    mergeSort(arr3);
    printArray("Sorted  ", arr3);
    cout << "Is sorted: ";
    print_bool(isSorted(arr3));
    cout << endl;
    
    // Test case 4: Array with duplicates
    vector<int> arr4 = {5, 2, 8, 2, 9, 1, 5, 5};
    cout << "Test Case 4: Array with Duplicates" << endl;
    printArray("Original", arr4);
    mergeSort(arr4);
    printArray("Sorted  ", arr4);
    cout << "Is sorted: ";
    print_bool(isSorted(arr4));
    cout << endl;
    
    // Test case 5: Single element
    vector<int> arr5 = {42};
    cout << "Test Case 5: Single Element" << endl;
    printArray("Original", arr5);
    mergeSort(arr5);
    printArray("Sorted  ", arr5);
    cout << "Is sorted: ";
    print_bool(isSorted(arr5));
    cout << endl;
    
    // Test case 6: Empty array
    vector<int> arr6 = {};
    cout << "Test Case 6: Empty Array" << endl;
    printArray("Original", arr6);
    mergeSort(arr6);
    printArray("Sorted  ", arr6);
    cout << "Is sorted: ";
    print_bool(isSorted(arr6));
    cout << endl;
    
    // Test case 7: Large array with negative numbers
    vector<int> arr7 = {-5, 3, -2, 8, -1, 0, 4, -3, 7};
    cout << "Test Case 7: Array with Negative Numbers" << endl;
    printArray("Original", arr7);
    mergeSort(arr7);
    printArray("Sorted  ", arr7);
    cout << "Is sorted: ";
    print_bool(isSorted(arr7));
    
    cout << endl << "All test cases completed!" << endl;
    
    return 0;
}