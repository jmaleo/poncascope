# -----------------------------------------------------------------------------
# ponca
# -----------------------------------------------------------------------------
# include(CPM)
# CPMAddPackage(
#   NAME ponca
#   GITHUB_REPOSITORY poncateam/ponca
#   GIT_TAG master
#   OPTIONS "PONCA_CONFIGURE_EXAMPLES OFF" "PONCA_CONFIGURE_DOC OFF" "PONCA_CONFIGURE_TESTS OFF"
# )

include(CPM)
CPMAddPackage(
  NAME ponca
  GITHUB_REPOSITORY jmaleo/ponca
  GIT_TAG waveJets
  OPTIONS "PONCA_CONFIGURE_EXAMPLES OFF" "PONCA_CONFIGURE_DOC OFF" "PONCA_CONFIGURE_TESTS OFF"
)
