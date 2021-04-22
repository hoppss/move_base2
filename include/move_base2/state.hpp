enum GlobalState
{
  G_UNACTIVE = 0,
  G_INITERROR,
  G_READY,
  G_RUNING,
};

enum NavState
{
  PLANNING = 0,
  CONTROLLING,
  EXCEPTION,
  STOPPING,
};