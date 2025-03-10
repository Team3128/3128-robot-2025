package frc.team3128.doglog;

public class DogLogOptions {
  private final boolean ntPublish;
  private final boolean captureNt;
  private final boolean captureDs;

  // Constructor
  public DogLogOptions(boolean ntPublish, boolean captureNt, boolean captureDs) {
      this.ntPublish = ntPublish;
      this.captureNt = captureNt;
      this.captureDs = captureDs;
  }

  // Getters
  public boolean ntPublish() { return ntPublish; }
  public boolean captureNt() { return captureNt; }
  public boolean captureDs() { return captureDs; }

  @Override
  public String toString() {
      return "DogLogOptions(ntPublish=" + ntPublish + ", captureNt=" + captureNt + ", captureDs=" + captureDs + ")";
  }

  // equals and hashCode need to be manually implemented too
}