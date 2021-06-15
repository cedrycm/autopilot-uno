enum CtrlFlags
{
    SEARCH_DELIVERY_SITE = 1,
    APPROACH_DELIVERY_SITE = 2,
    AVOID_COLLISION = 4,
    DELIVER_PACKAGE = 8,
    RECOVER = 16,
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