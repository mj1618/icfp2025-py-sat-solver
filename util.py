
def de_bruijn(k: int, n: int) -> str:
    """
    Generate a De Bruijn sequence for alphabet size k and subsequences of length n.

    Args:
        k (int): Size of the alphabet (e.g., 2 for binary, 10 for digits).
        n (int): Length of subsequences.

    Returns:
        str: A De Bruijn sequence as a string of digits in base k.
    """
    alphabet = range(k)
    a = [0] * (k * n)
    sequence = []

    def db(t: int, p: int):
        if t > n:
            if n % p == 0:
                sequence.extend(a[1:p+1])
        else:
            a[t] = a[t - p]
            db(t + 1, p)
            for j in range(a[t - p] + 1, k):
                a[t] = j
                db(t + 1, t)

    db(1, 1)
    return [alphabet[i] for i in sequence]


def chunks_loop(lst, chunk_size):
    result_chunks = []
    for i in range(0, len(lst), chunk_size):
        result_chunks.append(lst[i:i + chunk_size])
    return result_chunks
