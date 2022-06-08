#MinHeap class to keep track of all values + parent
class MinHeap:
    #Constructor for minheap
    def __init__(self):
        self.heap = []
        self.size =  0;

    def print(self):
        print("\nHEAP:")
        for i in self.heap:
            print(i)
        print("END OF HEAP\n")

    def getParentIndex(self, index):
        return (index - 1) // 2
    
    def getLeftChildIndex(self, index):
        return 2 * index + 1
    
    def getLeftChild(self, index):
        return self.heap[2 * index + 1]
    
    def getRightChildIndex(self, index):
        return 2 * index + 2
    
    def getRightChild(self, index):
        return self.heap[2 * index + 2]

    def hasLeftChild(self, index):
        return self.getLeftChildIndex(index) < self.size

    def hasRightChild(self, index):
        return self.getRightChildIndex(index) < self.size
    
    def swap(self, index1, index2):
        tempValue = self.heap[index2]
        self.heap[index2] = self.heap[index1]
        self.heap[index1] = tempValue
    
    #inserts tuples of format (fscore, gscore, node) in minheap, sorted on fscore then gscore then node
    def insert(self, value):
        #add node to end
        self.heap.append(value)
        self.size += 1

        #heapify up
        currIndex = self.size - 1
        while currIndex > 0:
            if self.heap[currIndex] >= self.heap[self.getParentIndex(currIndex)]:
                break
            self.swap(currIndex, self.getParentIndex(currIndex))
            currIndex = self.getParentIndex(currIndex)
    
    #removes the tuple with the smallest fscore (tiebreaker: smallest gscore of them if contains equal fscore) and returns vertex
    def pop(self):
        #already empty
        if self.size <= 0:
            return

        #move last element into root
        vertex = self.heap[0][2]
        self.heap[0] = self.heap[self.size - 1]
        self.heap.pop()
        self.size -= 1

        #heapify down
        currIndex = 0
        while self.hasLeftChild(currIndex):
            smallestNodeIndex = self.getLeftChildIndex(currIndex)
            if self.hasRightChild(currIndex):
                if self.getRightChild(currIndex) < self.getLeftChild(currIndex):
                   smallestNodeIndex = self.getRightChildIndex(currIndex)
            if self.heap[currIndex] <= self.heap[smallestNodeIndex]:
                break
            self.swap(currIndex, smallestNodeIndex)
            currIndex = smallestNodeIndex
        
        return vertex

    #returns 1 if node is in heap, 0 if not in heap
    def containsNode(self, node):
        for i in range(self.size):
            if node == self.heap[i][2]:
                return 1
        return 0
        
