#if !defined(RANGE_H)
#define RANGE_H

template <typename T>
class range {
	static_assert(std::is_integral<T>::value, "Ranges only work for int types");
public:
	class iterator {
		friend class range;
	public:
		T operator*() const { return i_; }
		const iterator& operator++() { ++i_; return *this; }
		iterator operator++(int) { iterator copy(*this); ++i_; return copy; }

		bool operator==(const iterator& rhs) { return i_ == rhs.i_; }
		bool operator!=(const iterator& rhs) { return i_ != rhs.i_; }

	protected:
		iterator(T i) : i_(i) { }

	private:
		T i_;
	};

	iterator begin() const { return begin_; }
	iterator end() const { return end_; }
	range(T begin, T end) : begin_(begin), end_(end) { }

private:
	iterator begin_;
	iterator end_;
};

template <typename T>
typename range<T>::iterator begin(const range<T>& range)
{
	return range.begin();
}

template <typename T>
typename range<T>::iterator end(const range<T>& range)
{
	return range.end();
}

#endif // RANGE_H