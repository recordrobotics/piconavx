import gc
import ustruct
import host_protocol

class BufferChunk:
    def __init__(self, size:int):
        self.buffer = bytearray(size)
        self.buffer_size = size
        self.buffer_view = memoryview(self.buffer)
        self.next:BufferChunk|None = None
        self.prev:BufferChunk|None = None
        
class RingBuffer:
    def __init__(self, size:int, chunk_size:int):
        self.chunk_count = int(size / chunk_size)
        self.unused_chunks = [BufferChunk(chunk_size) for _ in range(self.chunk_count)]
        self.unused_chunks_ptr = self.chunk_count - 1
        self.start:BufferChunk|None = None
        self.end:BufferChunk|None = None
        
    def reset(self):
        self.end = None
        node = self.start
        while node is not None:
            self.unused_chunks_ptr += 1
            self.unused_chunks[self.unused_chunks_ptr] = node
            node = node.next            
        self.start = None
    
    def has_unused(self):
        return self.unused_chunks_ptr >= 0
    
    def use_chunk(self):
        chunk = self.unused_chunks[self.unused_chunks_ptr]
        self.unused_chunks_ptr -= 1
        chunk.next = None
        if self.start is None:
            self.start = chunk
        if self.end is not None:
            self.end.next = chunk
            chunk.prev = self.end
        else:
            chunk.prev = None
        self.end = chunk
        return chunk.buffer_view
    
    # Relinks chunk to the end of the list
    def renew_chunk(self, chunk:BufferChunk):
        if chunk.prev is not None:
            chunk.prev.next = chunk.next
        if chunk.next is not None:
            chunk.next.prev = chunk.prev
        chunk.next = None
        if self.end is not None:
            self.end.next = chunk
            chunk.prev = self.end
        else:
            chunk.prev = None
        self.end = chunk

class Feed:
    def __init__(self):
        free = gc.mem_free()
        array_size: int = 15000 # 15 kB by default
        if free != -1:
            if free < array_size:
                array_size = int(free / 8) # 1/8th of the free memory if 15kB does not fit
            else:
                array_size = max(array_size, int(free / 8)) # 1/8th of the free memory or 15kB

        self.chunk_size = 92
        self.buffer = RingBuffer(array_size, self.chunk_size)
        self.enabled = False
        self.feed_overflow = host_protocol.SET_FEED_OVERFLOW_DELETE_OLDEST
        self.reduced_frequency_denominator = 4
        self.reduced_frequency_numerator = 1
        
    def enable(self):
        self.enabled = True
    
    def disable(self):
        self.enabled = False
    
    def accepting(self)->bool:
        return self.enabled
    
    def overflow(self)->memoryview|None:
        if self.feed_overflow == host_protocol.SET_FEED_OVERFLOW_SKIP:
            return None # ignore new data
        elif self.feed_overflow == host_protocol.SET_FEED_OVERFLOW_DELETE_OLDEST:
            chunk = self.buffer.start
            if chunk is not None:
                self.buffer.renew_chunk(chunk)
                return chunk.buffer_view
        elif self.feed_overflow == host_protocol.SET_FEED_OVERFLOW_REDUCE_OLDEST_FREQUENCY:
            index = int(self.buffer.chunk_count / self.reduced_frequency_denominator * self.reduced_frequency_numerator)
            self.reduced_frequency_numerator += 1
            if self.reduced_frequency_numerator == self.reduced_frequency_denominator:
                if self.reduced_frequency_denominator < 256:
                    self.reduced_frequency_denominator *= 2
                self.reduced_frequency_numerator = 1
            ctr = 0
            node = self.buffer.start
            while ctr < index and node is not None:
                node = node.next
                ctr+=1
            if ctr == index and node is not None:
                self.buffer.renew_chunk(node)
                return node.buffer_view
        return None
    
    def submit(self, timestamp: int)->memoryview | None:
        mv:memoryview|None = None
        if not self.buffer.has_unused():
            mv = self.overflow()
            if mv is None:
                return None
        else:
            self.reduced_frequency_denominator = 4
            self.reduced_frequency_numerator = 1
        if mv is None:
            mv = self.buffer.use_chunk()
        ustruct.pack_into(">I", mv, 0, timestamp)
        return mv