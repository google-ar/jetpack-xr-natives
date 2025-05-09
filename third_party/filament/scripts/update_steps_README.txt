Filament Update Steps

1 Filament team needs to create a tag including the change to be synced
  (go/gh/google/filament/releases has a list)
2 Make sure the date stamp (e.g. 20190626) is common to all the binary artifacts
  (there is a lag time between tags appearing and assets appearing.)
3 Open a clean google3 client, sync to head
4 SETUP: get copybara: go/copybara
  Run copybara, e.g.:
    copybara third_party/filament/copy.bara.sky --destination-cl 256219331 --piper-description-behavior OVERWRITE default tags/v1.3.2
  DEBUG: If the patches fail to apply, see go/copybara-patch-notes
5 SETUP: If you don't already have one, make a scratch directory in your home folder:
    mkdir ~/dev
    mkdir ~/dev/scratch
  Get the prebuilt binaries using the get_filament_binaries script:
    third_party/filament/scripts/get_filament_binaries.sh v1.3.2 20181213 ~/dev/scratch
6 Make a CL and run presubmits
  OPTIONAL: third_party/filament/scripts/filaudit.sh runs several builds and tests
  that have been broken by filament upgrades in the past.  Most of these should be
  in the presubmits.
7 Run agmm scuba tests that are too expensive for their presubmits
  https://g3doc.corp.google.com/java/com/google/android/apps/gmm/ar/sceneform/assetdata/README.md?cl=head#running-the-sceneform-scuba-tests
8 Update Scene Viewer static assets (if needed) go/fastglance/updating


Before Submit:
Send an email to GeoAR Experiences Eng <geoar-experiences-eng@google.com> and impress-team <impress-team@google.com> to avoid rollbacks

Note: this was adapted from https://bit.googleplex.com/#/amperez/5834473824321536
