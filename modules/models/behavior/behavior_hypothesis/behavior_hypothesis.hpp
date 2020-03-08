



class BehaviorHypothesis : public BehaviorActionStore {
  virtual Probability GetProbability(ObservedWorld, Action) const = 0;
}