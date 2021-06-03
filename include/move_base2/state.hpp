enum NavState
{
  UNACTIVE = 0,
  READY,
  PLANNING,
  CONTROLLING,
  WAITING,
  EXCEPTION,
  STOPPING,
};

enum NavMode
{
  NavMode_AB = 0,
  NavMode_Track,
  NavMode_STOP,
};