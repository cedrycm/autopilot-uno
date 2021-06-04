enum CtrlFlags
{
    APPROACH_TARGET = 1,
    AVOID_COLLISION = 2,
    DELIVER_PACKAGE = 4,
    RECOVER = 8,
};

inline CtrlFlags operator|(CtrlFlags a, CtrlFlags b)
{
    return static_cast<CtrlFlags>(static_cast<int>(a) | static_cast<int>(b));
}

inline CtrlFlags operator&(CtrlFlags a, CtrlFlags b)
{
    return static_cast<CtrlFlags>(static_cast<int>(a) & static_cast<int>(b));
}

inline CtrlFlags operator^(CtrlFlags a, CtrlFlags b)
{
    return static_cast<CtrlFlags>(static_cast<int>(a) ^ static_cast<int>(b));
}