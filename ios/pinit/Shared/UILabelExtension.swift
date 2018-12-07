import UIKit

extension UILabel {
    
    public func makeLabelCircular() {
        self.layer.cornerRadius = self.layer.frame.width / 2.0
    }
    
}
