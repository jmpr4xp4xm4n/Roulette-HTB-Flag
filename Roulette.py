from arybo.lib import MBA
import random
from cffi import FFI


host = '159.65.92.160'
port = 30583


ffi = FFI()
# libm4ri definitions
ffi.cdef('''
// misc.h
typedef int BIT;
typedef int rci_t;
typedef int wi_t;
typedef uint64_t word;

// mzd.h
typedef struct {
  size_t size; /*!< number of words */
  word* begin; /*!< first word */
  word* end; /*!< last word */
} mzd_block_t;

typedef struct mzd_t {
  rci_t nrows;  /*!< Number of rows. */ 
  rci_t ncols;  /*!< Number of columns. */
  wi_t  width;  /*!< Number of words with valid bits: width = ceil(ncols / m4ri_radix) */
  wi_t rowstride;
  wi_t offset_vector;
  wi_t row_offset;   /*!< Number of rows to the first row counting from the start of the first block. */
  uint8_t flags;
  uint8_t blockrows_log;
  word high_bitmask;    /*!< Mask for valid bits in the word with the highest index (width - 1). */
  mzd_block_t *blocks;  /*!< Pointers to the actual blocks of memory containing the values packed into words. */
  word   **rows;        /*!< Address of first word in each row, so the first word of row i is is m->rows[i] */
  uint64_t dummy;       /*!< ensures sizeof(mzd_t) == 64 */
} mzd_t;

mzd_t *mzd_init(rci_t const r, rci_t const c);
mzd_t *mzd_copy(mzd_t *DST, mzd_t const *A);
int mzd_solve_left(mzd_t *A, mzd_t *B, int const cutoff, int const inconsistency_check);
''')
m4ri = ffi.dlopen('libm4ri.so')

def mzd_read_bit(M, row, col):
	return (M.rows[row][col//64] >> (col%64)) & 1

def mzd_write_bit(M, row, col, value):
	spot = col % 64
	w = M.rows[row][col//64]
	w = w & ~(1 << spot) | (value << spot)
	M.rows[row][col//64] = w


N = 128
M = 30
b = 32
MAGIC = 0xb249b015
mask = (1 << b) - 1

class Twister:
	def rol(self, x, d):
		ans = (x << d) & mask
		ans = ans | (x >> (b - d))
		return ans & mask

	def __init__(self, state=None):
		self.index = N
		if state is not None:
			assert len(state) == N
			self.STATE = state[:]
		else:
			self.STATE = [0] * N
			for i in range(N):
				self.STATE[i] ^= random.getrandbits(32)

	def twist(self):
		for i in range(N):
			self.STATE[i] ^= self.rol(self.STATE[(i+1) % N], 3)
			self.STATE[i] ^= self.rol(self.STATE[(i+M) % N], b - 9)
			self.STATE[i] ^= MAGIC

	def rand(self):
		if self.index >= N:
			self.twist()
			self.index = 0
		y = self.STATE[self.index]
		y ^= self.rol(y, 7)
		y ^= self.rol(y, b - 15)
		self.index += 1

		return y & mask


# read spins from server
num_spins = int(N*b//5)+1
import socket
while True:
	spins = []
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))
	sock.sendall(b'1\n1\n'*num_spins)  # bet 1 on even
	buf = ''
	count = 0
	while count < num_spins:
		buf += sock.recv(1024).decode('ascii')
		if not buf:
			break
		while '\n' in buf:
			i = buf.index('\n')
			msg = buf[:i]
			buf = buf[i+1:]
			if 'stops at' in msg:
				spins.append(int(msg[msg.index('at')+ 3:]))
				count += 1
	print('Got', len(spins), 'spins')
	if len(spins) >= num_spins:
		break
	sock.close()

# construct system of equations
mba = MBA(b)
A = m4ri.mzd_init(N*b, N*b)
rhs = m4ri.mzd_init(N*b, 1)
sym_random = Twister([mba.var('%d_' % i) for i in range(N)])  # twister with symbolic variables
k = 0
for i in range(num_spins):
	sym_rand = sym_random.rand() & 31
	server_rand = spins[i]
	for j in range(5):  # each bit
		expr = (str(sym_rand[j])[1:-1].split(' + '))
		rhs_k = 0
		for term in expr:
			if term == '1':
				# move constant term to rhs
				rhs_k = 1
			else:
				si, bi = term.split('_')
				si, bi = int(si), int(bi)
				mzd_write_bit(A, k, b*si + bi, 1)
		rhs_k ^= server_rand & 1
		mzd_write_bit(rhs, k, 0, rhs_k)
		server_rand >>= 1
		k += 1
		if k == N*b:
			break

# solve A*state = rhs
Acopy = m4ri.mzd_init(N*b, N*b)
state = m4ri.mzd_init(N*b, 1)
m4ri.mzd_copy(Acopy, A)
m4ri.mzd_copy(state, rhs)
m4ri.mzd_solve_left(Acopy, state, 0, 1)
rec_state = []
for i in range(N):
	s = 0
	for j in range(b):
		s += mzd_read_bit(state, b*i + j, 0) << j
	rec_state.append(s)

# twister with the recovered state
random = Twister(rec_state)
# step our RNG to sync with the server
for i in range(num_spins):
	random.rand()

# send number bets
import time
bet = 1
for i in range(14):
	sock.sendall('3\n{}\n{}\n'.format(random.rand()&31, bet).encode('ascii'))
	time.sleep(0.01)
	print(sock.recv(1024).decode('ascii'))
	bet *= 10
sock.close()
