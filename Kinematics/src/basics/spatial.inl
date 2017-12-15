
/*
 * spatial.inl
 *
 * Templated defininitions of spatial data structures
 *
 * Author: JochenAlt
 */

	
template<class T>
PentaType<T>::PentaType() {
	null();
}

template<class T>
PentaType<T>::PentaType(const PentaType& par) {
	for (int i = 0;i<NumberOfLegs;i++)
		a[i] = par.a[i];
}

template<class T>
void PentaType<T>::operator=(const PentaType& par) {
	for (int i = 0;i<NumberOfLegs;i++)
		a[i] = par.a[i];
}

template<class T>
bool PentaType<T>::operator==(const PentaType& par) {
	for (int i = 0;i<NumberOfLegs;i++)
		if (fabs(a[i] != par.a[i]))
			return false;
	return true;
}

template<class T>
bool PentaType<T>::operator!=(const PentaType& pos) {
	return !((*this) == pos);
};

template<class T>
T& PentaType<T>::operator[](int idx) {
	if ((idx >= 0) || ( idx < NumberOfLegs))
		return a[idx];
	static T dummy;
	return dummy;
}

template<class T>
const T& PentaType<T>::operator[](int idx) const {
	if ((idx >= 0) || ( idx < NumberOfLegs))
		return a[idx];
	static T dummy;
	return dummy;
}

template<class T>
void PentaType<T>::null() {
	for (int i = 0;i<NumberOfLegs;i++)
		a[i].null();
}

template<class T>
bool PentaType<T>::isNull() {
	for (int i = 0;i<NumberOfLegs;i++)
		if (a[i].isNull())
			return false;
	return true;
}

template<class T>
PentaType<T> PentaType<T>::getRotatedAroundZ(angle_rad ori) const {
	PentaType<T> result;
	for (int i = 0;i<NumberOfLegs;i++)
		result.a[i] = a[i].getRotatedAroundZ(ori);
	return result;
}


template<class T>
std::ostream& PentaType<T>::serialize(std::ostream &out) const {
	out << "{ \"a\":";
	serializeArrayOfSerializable(a, NumberOfLegs, out);
	out << "}";
	return out;
}

template<class T>
std::istream& PentaType<T>::deserialize(std::istream &in, bool&ok) {
    if (in) {
    	bool ok = true;
    	parseCharacter(in, '{', ok);
    	parseString(in, ok); // "a"
    	parseCharacter(in, ':', ok);
    	int len;
    	deserializeArrayOfSerializable(in, a, len, ok);
    	parseCharacter(in, '}', ok);
    }
    return in;
}

