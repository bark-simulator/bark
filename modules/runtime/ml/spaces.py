from gym import Space
import numpy as np

class Discrete(Space):
    """A discrete space in :math:`\{ 0, 1, \dots, n-1 \}`.

    Example::

        >>> Discrete(2)

    """
    def __init__(self, n):
        self.n = n
        super(Discrete, self).__init__((), np.int64)

    def sample(self):
        return self.np_random.randint(self.n)

    def contains(self, x):
        if isinstance(x, int):
            as_int = x
        elif isinstance(x, (np.generic, np.ndarray)) and (x.dtype.kind in np.typecodes['AllInteger'] and x.shape == ()):
            as_int = int(x)
        else:
            return False
        return as_int >= 0 and as_int < self.n

    def __repr__(self):
        return "Discrete(%d)" % self.n

    def __eq__(self, other):
        return isinstance(other, Discrete) and self.n == other.n


class BoundedContinuous(Space):
  """A continuous space

  Example::

      >>> BoundedContinuous(2)

  """
  def __init__(self,
                n,
                low=None,
                high=None):
    self._n = n
    self._low = low or -10000.0
    self._high = high or 100000.0
    Space.__init__(self, shape=(n,))

  def sample(self):
    if len(self._low) > 1 and len(self._high) > 1:
      # assert(len(self._low) == len(self._high))
      sample_vec = []
      for mi, ma in zip(self._low, self._high):
        sample_vec.append(self.np_random.uniform(mi, ma))
      return np.hstack(sample_vec)
    return self.np_random.uniform(size=(self._n,))

  @property
  def low(self):
    return self._low

  @property
  def high(self):
    return self._high

  def __repr__(self):
      return "BoundedContinuous(%d)" % self._n

  def __eq__(self, other):
    return isinstance(other, BoundedContinuous) and self._n == other._n
