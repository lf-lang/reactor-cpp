

#include "reactor-cpp/scops.hh"

template<class T>
void reactor::MutableScope<T>::begin_transaction() {
  transaction_.reset();
}

template<class T>
void reactor::MutableScope<T>::end_transaction() {
  transaction_.execute();
}