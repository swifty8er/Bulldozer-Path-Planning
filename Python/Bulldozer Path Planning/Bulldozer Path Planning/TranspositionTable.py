import random


class TranspositionTable:
    #A transposition table used to store the status of game states of the playing field


    def _createZorbistKey(self):
        threshold = 2**self._NUM_OF_BITS - 1
        #unique may take too long
        node_num_id = dict()
        #find a unique number for vehicle at first node
        node_num_id["vehicle"] = random.randint(1,threshold)
        #find a unique number for disk at first node
        node_num_id["disk"] = random.randint(1,threshold)
        while (node_num_id["disk"] == node_num_id["vehicle"]):
            node_num_id["disk"] = random.randint(1,threshold)
        
        #find a unique number for empty at first node
        node_num_id["empty"] = random.randint(1,threshold)
        while ((node_num_id["empty"] == node_num_id["vehicle"]) or (node_num_id["empty"] == node_num_id["disk"])):
            node_num_id["empty"] = random.randint(1,threshold)
        
        # store in Zorbist Key
        Map_num_ids = [node_num_id.copy()]
        # keep track of all current numbers
        rand_nums = [node_num_id["vehicle"], node_num_id["disk"], node_num_id["empty"]]
    
        for i in range(2, self._num_of_nodes):
            #find a unique number for vehicle at firt node
            node_num_id["vehicle"] = random.randint(1,threshold)
            while ((node_num_id["vehicle"] in rand_nums) == True):
                node_num_id["vehicle"] = random.randint(1,threshold)
            
            rand_nums.append(node_num_id["vehicle"])
            #find a unique number for disk at firt node
            node_num_id["disk"] = random.randint(1,threshold)
            while ((node_num_id["disk"] in rand_nums) == True):
                node_num_id["disk"] = random.randint(1,threshold)
            
            rand_nums.append(node_num_id["disk"])
            #find a unique number for vehicle at firt node
            node_num_id["empty"] = random.randint(1,threshold)
            while ((node_num_id["empty"] in rand_nums) == True):
                node_num_id["empty"] = random.randint(1,threshold)
            
            rand_nums.append(node_num_id["empty"])
            Map_num_ids.append(node_num_id.copy())
        
    
        return Map_num_ids

    
    def __init__(self, num_of_nodes, num_of_bits, size):
        self._NUM_OF_BITS = num_of_bits #32
        self._num_of_nodes = num_of_nodes
        # Create Zorbist Key
        self._zorbist_key = self._createZorbistKey()
        #set up the transposition table
        self._size = 1<<size+9 #try 23 if not opimal
        empty_node = dict()
        empty_node["cf"] = 0
        empty_node["key"] = 0
        empty_node["visited"] = False
        node_array = [empty_node]
        self._table = [node_array]*self._size


    def getZorbistKey(self, node):

        key = 0
        for i in range(len(self._zorbist_key)):
            #find the current value for a certain node
            if (node.vehicle_pos == i):
                curr_value = self._zorbist_key[i]["vehicle"]
            elif ((i in node.disk_poses) == True):
                curr_value = self._zorbist_key[i]["disk"]
            else:
                curr_value = self._zorbist_key[i]["empty"]
            #add to current key value
            key = key^curr_value

        return key


    def addToTable(self, node):
        # Get the Zorbist Key for a state
        key = self.getZorbistKey(node)
        index = key%self._size
    
        #check if a value already exists
        if (self._table[index][0]["key"] == 0):
            self._table[index][0]["key"] = key
            self._table[index][0]["cf"] = node.cf
            status = 'E'
        else:
            #check if current node is the same as stored node(s)
            i = 0
            found = False
            while ((i < len(self._table[index])) and (found == False)):
                if (key == self._table[index][i]["key"]):
                    found = True
                    if (node.cf < self._table[index][i]["cf"]):
                        self._table[index][i]["cf"] = node.cf
                        status = 'R'
                    else:
                        status = 'N'
                    
                
                i += 1
            
            if (found == False):
                new_node = dict()
                new_node["key"] = key
                new_node["cf"] = node.cf
                new_node["visited"] = False
                self._table[index].append(new_node)
                status = 'E'

        return status

    def isVisited(self, node, set_to_visit):
        key = self.getZorbistKey(node)
        index = key%self._size
        found = False
        visited = False
        i = 0
        while ((i < len(self._table[index])) and (found == False)):
            if (key == self._table[index][i]["key"]):
                found = True
                if (self._table[index][i]["visited"] == False):
                    if (set_to_visit == True):
                        self._table[index][i]["visited"] = True
                else:
                     visited = True
            i += 1

        return visited
