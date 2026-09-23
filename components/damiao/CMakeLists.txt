# can_bus への依存は idf_component.yml で宣言している (コンポーネントマネージャが REQUIRES に追加する)
idf_component_register(
    SRCS "src/damiao.c"
    INCLUDE_DIRS "include"
)
